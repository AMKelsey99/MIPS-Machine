module reg_file (RR1,RR2,WR,WD,RegWrite,RD1,RD2,clock);
  input [1:0] RR1,RR2,WR;
  input [15:0] WD;
  input RegWrite,clock;
  output [15:0] RD1,RD2;
  reg [15:0] Regs[0:3];
  assign RD1 = Regs[RR1];
  assign RD2 = Regs[RR2];
  initial Regs[0] = 0;
  always @(negedge clock)
		if (RegWrite==1 & WR!=0)
	Regs[WR] <= WD;
endmodule

//1-bit Adder
module half_adder(a,b,S,C);
	input a,b;
	output S,C;
	xor (S,a,b);
	and (C,a,b);
endmodule
module full_adder(a,b,in,S,Cout);
	input a,b,in;
	output S,Cout;
	half_adder ha1 (a,b,S0,C0);
	half_adder ha2 (S0,in,S,C1);
	or (Cout,C0,C1);
endmodule

//2x1 Multiplexer
module mux2x1 (x, y, S, out);
	input x, y, S;
	output out;
	and output_1 (outx, ~S, x);
	and output_2 (outy, S, y);
	or output_3 (out, outx, outy);
endmodule
//4x1 Multiplexer
module mux4x1(I, S, out);
	input [3:0] I;
	input [1:0] S;
	output out;
	wire [3:0] P;
	and (P[3], I[3], S[1], S[0]);
	and (P[2], I[2], S[1], ~S[0]);
	and (P[1], I[1], ~S[1], S[0]);
	and (P[0], I[0], ~S[1], ~S[0]);
	or  (out, P[0], P[1], P[2], P[3]);
endmodule
module mux16x2x1(X, Y, S, Out);
    input [15:0] X, Y;
    input S;
    output [15:0] Out;
    mux2x1      mux0(X[0], Y[0], S, Out[0]),
                mux1(X[1], Y[1], S, Out[1]),
                mux2(X[2], Y[2], S, Out[2]),
                mux3(X[3], Y[3], S, Out[3]),
                mux4(X[4], Y[4], S, Out[4]),
                mux5(X[5], Y[5], S, Out[5]),
                mux6(X[6], Y[6], S, Out[6]),
                mux7(X[7], Y[7], S, Out[7]),
                mux8(X[8], Y[8], S, Out[8]),
                mux9(X[9], Y[9], S, Out[9]),
                mux10(X[10], Y[10], S, Out[10]),
                mux11(X[11], Y[11], S, Out[11]),
                mux12(X[12], Y[12], S, Out[12]),
                mux13(X[13], Y[13], S, Out[13]),
                mux14(X[14], Y[14], S, Out[14]),
                mux15(X[15], Y[15], S, Out[15]);
endmodule
module ALU_1_bit(A, B, ctrl, invA, negB, Cin, less, Cout, Result);
	input [1:0] ctrl;
	input A, B, invA, negB, Cin, less;
	output Cout, Result;
	//P[0] - AND/NAND
	//P[1] - OR/NOR
	//P[2] - Adder
	//P[3] - Less
	wire [3:0] P;
	//Inverts A if ctrl[0]/invA is high
	xor (a,invA,A);
	//Inverts B if ctrl[1]/subtract/invB is high
	xor (b,negB,B);
	//NAND or NOR if invA and invB is high
	and (P[0],a,b);
	or (P[1],a,b);
	//Compliments B by adding if Cin is high
	full_adder adder (a,b,Cin,P[2],Cout);
	//Sets P[3] to less
	assign P[3] = less;

	mux4x1 mux(P, ctrl, Result);
endmodule

module ALU3_1_bit(A, B, ctrl, invA, negB, Cin, Result, Set, Overflow);
	input [1:0] ctrl;
	input A, B, invA, negB, Cin;
	output Result, Set, Overflow;
	//P[0] - AND/NAND
	//P[1] - OR/NOR
	//P[2] - Adder
	//P[3] - Less
	wire [3:0] P;
	wire a, b;
	//Inverts A if ctrl[0]/invA is high
	xor (a,invA,A);
	//Inverts B if ctrl[1]/subtract/invB is high
	xor (b,negB,B);
	//NAND or NOR if invA and invB is high
	and (P[0],a,b);
	or (P[1],a,b);

	full_adder adder (a,b,Cin,P[2],Cout);
	//Sets P[3] to 0
	assign P[3] = 0;
	//Selects result
	mux4x1 mux(P, ctrl, Result);
	//Detect overflow
	xor (Overflow, Cin, Cout);
	//Assigns the sign bit of addition result as set
	assign Set = P[2];
endmodule

module ALU (op, A, B, Result, Zero);
	input [3:0] op;
	input [15:0] A, B;
	output [15:0] Result;
	output Zero;
	//Connect 1-bit ALUs
	ALU_1_bit alu0 (A[0], B[0], op[1:0], op[3], op[2], op[2], less, C0, Result[0]),
					  alu1 (A[1], B[1], op[1:0], op[3], op[2], C0, 1'b0, C1, Result[1]),
					  alu2 (A[2], B[2], op[1:0], op[3], op[2], C1, 1'b0, C2, Result[2]),
					  alu3 (A[3], B[3], op[1:0], op[3], op[2], C2, 1'b0, C3, Result[3]),
					  alu4 (A[4], B[4], op[1:0], op[3], op[2], C3, 1'b0, C4, Result[4]),
					  alu5 (A[5], B[5], op[1:0], op[3], op[2], C4, 1'b0, C5, Result[5]),
					  alu6 (A[6], B[6], op[1:0], op[3], op[2], C5, 1'b0, C6, Result[6]),
					  alu7 (A[7], B[7], op[1:0], op[3], op[2], C6, 1'b0, C7, Result[7]),
					  alu8 (A[8], B[8], op[1:0], op[3], op[2], C7, 1'b0, C8, Result[8]),
					  alu9 (A[9], B[9], op[1:0], op[3], op[2], C8, 1'b0, C9, Result[9]),
					  alu10 (A[10], B[10], op[1:0], op[3], op[2], C9, 1'b0, C10, Result[10]),
					  alu11 (A[11], B[11], op[1:0], op[3], op[2], C10, 1'b0, C11, Result[11]),
					  alu12 (A[12], B[12], op[1:0], op[3], op[2], C11, 1'b0, C12, Result[12]),
					  alu13 (A[13], B[13], op[1:0], op[3], op[2], C12, 1'b0, C13, Result[13]),
					  alu14 (A[14], B[14], op[1:0], op[3], op[2], C13, 1'b0, C14, Result[14]);
	ALU3_1_bit alu15(A[15], B[15], op[1:0], op[3], op[2], C14, Result[15], less, Ovrflw);

	//Zero detection
	nor (Zero, Result[0], Result[1], Result[2], Result[3], Result[4], Result[5], Result[6], Result[7], Result[8], Result[9], Result[10], Result[11], Result[12], Result[13], Result[14], Result[15]);
endmodule

module MainControl (Op,Control); 
  input [3:0] Op;
  output reg [10:0] Control;
// RegDst,ALUSrc,MemtoReg,RegWrite,MemWrite,Beq,Bne,ALUctl
  always @(Op) case (Op)
    4'b0000: Control <= 11'b10010_00_0010; // ADD
	4'b0001: Control <= 11'b10010_00_0110; // SUB
	4'b0010: Control <= 11'b10010_00_0000; // AND
	4'b0011: Control <= 11'b10010_00_0001; // OR
	4'b0100: Control <= 11'b10010_00_1101; // NAND
	4'b0101: Control <= 11'b10010_00_1100; // NOR
	4'b0110: Control <= 11'b10010_00_0111; // slt
    4'b0111: Control <= 11'b01010_00_0010; // addi
    4'b1000: Control <= 11'b01110_00_0010; // lw
    4'b1001: Control <= 11'b01001_00_0010; // sw
    4'b1010: Control <= 11'b00000_10_0110; // beq
    4'b1011: Control <= 11'b00000_01_0110; // bne
  endcase
endmodule

module CPU (clock,PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);
  input clock;
  output [15:0] PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD;

  initial begin 
 // Program: swap memory cells and compute absolute value
    IMemory[0] = 16'b1000_00_01_00000000;  // lw $t1, 0($0) 
    IMemory[1] = 16'b1000_00_10_00000010;  // lw $t2, 2($0)
    IMemory[2] = 16'b0000000000000000;  // nop
    IMemory[3] = 16'b0000000000000000;  // nop
    IMemory[4] = 16'b0000000000000000;  // nop
    IMemory[5] = 16'b0110_01_10_11_000000; // slt $t3, $t1, $t2
    IMemory[6] = 16'b0000000000000000;  // nop
    IMemory[7] = 16'b0000000000000000;  // nop
    IMemory[8] = 16'b0000000000000000;  // nop
    IMemory[9] = 16'b1010_11_00_00000101;  // beq $t3, $0, IMemory[15]
    IMemory[10] = 16'b0000000000000000;  // nop
    IMemory[11] = 16'b0000000000000000;  // nop
    IMemory[12] = 16'b0000000000000000;  // nop
    IMemory[13] = 16'b1001_00_01_00000010;  // sw $t1, 2($0)
    IMemory[14] = 16'b1001_00_10_00000000;  // sw $t2, 0($0) 
    IMemory[15] = 16'b0000000000000000;  // nop
    IMemory[16] = 16'b0000000000000000;  // nop
    IMemory[17] = 16'b0000000000000000;  // nop
    IMemory[18] = 16'b1000_00_01_00000000;  // lw $t1, 0($0)  
    IMemory[19] = 16'b1000_00_10_00000010;  // lw $t2, 2($0) 
    IMemory[20] = 16'b0000000000000000;  // nop
    IMemory[21] = 16'b0000000000000000;  // nop
    IMemory[22] = 16'b0000000000000000;  // nop
    IMemory[23] = 16'b0101_10_10_10_000000;  // nor $t2, $t2, $t2 (sub $3, $1, $2 in twos complement)
    IMemory[24] = 16'b0000000000000000;  // nop
    IMemory[25] = 16'b0000000000000000;  // nop
    IMemory[26] = 16'b0000000000000000;  // nop
    IMemory[27] = 16'b0111_10_10_00000001;  // addi $t2, $t2, 1 
    IMemory[28] = 16'b0000000000000000;  // nop
    IMemory[29] = 16'b0000000000000000;  // nop
    IMemory[30] = 16'b0000000000000000;  // nop
    IMemory[31] = 16'b0000_10_01_11_000000;   // add $t3, $t1, $t2 
    
    
 // Data
    DMemory [0] = 16'd5; // swap the cells and see how the simulation output changes
    DMemory [1] = 16'd7;
  end
  
  // Pipeline 
    // IF 
    wire [15:0] PCplus2, NextPC;
    reg[15:0] PC, IMemory[0:1023], IFID_IR, IFID_PCplus2;
    ALU fetch (4'b0010,PC,16'b0010,PCplus2,Unused1);
    //Branch Mux
    and (beqZero, EXMEM_Beq, EXMEM_Zero);
    and (bneNotZero, EXMEM_Bne, ~EXMEM_Zero);
    or (branch, beqZero, bneNotZero);
    mux16x2x1 muxBranch   (PCplus2, EXMEM_Target, branch, NextPC);
    
    // ID
    wire [10:0] control;
    reg IDEX_RegWrite,IDEX_MemtoReg,
        IDEX_Beq, IDEX_Bne,  IDEX_MemWrite,
        IDEX_ALUSrc,  IDEX_RegDst;
    reg [3:0] IDEX_ALUctl;
    wire [15:0] RD1,RD2,SignExtend, WD;
    reg [15:0] IDEX_PCplus2,IDEX_RD1,IDEX_RD2,IDEX_SignExt;
    reg [15:0] IDEX_IR; // For monitoring the pipeline
    reg [1:0]  IDEX_rt,IDEX_rd;
    reg MEMWB_RegWrite; // part of MEM stage, but declared here before use (to avoid error)
    reg [1:0] MEMWB_rd; // part of MEM stage, but declared here before use (to avoid error)
    
    // reg_file (RR1[2],RR2[2],WR[2],WD[16],RegWrite,RD1[16],RD2[16],clock);
    reg_file rf (IFID_IR[11:10],IFID_IR[9:8],MEMWB_rd,WD,MEMWB_RegWrite,RD1,RD2,clock);
    // MainControl (Op[4],Control[11]); {RegDst,ALUSrc,MemtoReg,RegWrite,MemWrite,Beq,Bne,ALUctl}
    MainControl MainCtr (IFID_IR[15:12], control);
    assign SignExtend = {{8{IFID_IR[7]}},IFID_IR[7:0]}; // sign extension unit

    // EXE
        reg EXMEM_RegWrite,EXMEM_MemtoReg,
            EXMEM_Beq, EXMEM_Bne,  EXMEM_MemWrite;
        wire [15:0] Target;
        reg EXMEM_Zero;
        reg [15:0] EXMEM_Target,EXMEM_ALUOut,EXMEM_RD2;
        reg [15:0] EXMEM_IR; // For monitoring the pipeline
        reg [1:0] EXMEM_rd;
        wire [15:0] B,ALUOut;
        wire [3:0] ALUctl;
        wire [1:0] WR;
        // ALU (op[4], A[16], B[16], Result[16], Zero);
        ALU branch (4'b0010,IDEX_SignExt<<1,IDEX_PCplus2,Target,Unused2);
        ALU ex (IDEX_ALUctl, IDEX_RD1, B, ALUOut, Zero);
        
        mux16x2x1 muxB (IDEX_RD2, IDEX_SignExt, IDEX_ALUSrc, B);        // ALUSrc Mux 
        mux2x1	muxWR0 (IDEX_rt[0],IDEX_rd[0], IDEX_RegDst, WR[0]),        // RegDst Mux
  			    muxWR1 (IDEX_rt[1],IDEX_rd[1], IDEX_RegDst, WR[1]);

    // MEM
        reg MEMWB_MemtoReg;
        reg [15:0] DMemory[0:1023],MEMWB_MemOut,MEMWB_ALUOut;
        reg [15:0] MEMWB_IR; // For monitoring the pipeline
        wire [15:0] MemOut;
        assign MemOut = DMemory[EXMEM_ALUOut>>1];
        always @(negedge clock) if (EXMEM_MemWrite) DMemory[EXMEM_ALUOut>>1] <= EXMEM_RD2;
    // WB
        mux16x2x1 muxWD (MEMWB_ALUOut, MEMWB_MemOut, MEMWB_MemtoReg, WD); // MemtoReg Mux
    
       initial begin
        PC = 0;
        // Initialize pipeline registers
        IDEX_RegWrite=0;IDEX_MemtoReg=0;IDEX_Beq=0;IDEX_Bne=0;IDEX_MemWrite=0;IDEX_ALUSrc=0;IDEX_RegDst=0;IDEX_ALUSrc=0;
        IFID_IR=0;
        EXMEM_RegWrite=0;EXMEM_MemtoReg=0;EXMEM_Beq=0;EXMEM_Bne=0;EXMEM_MemWrite=0;
        EXMEM_Target=0;
        MEMWB_RegWrite=0;MEMWB_MemtoReg=0;;
       end
    
    // Running the pipeline
       always @(negedge clock) begin 
    // IF
        PC <= NextPC;
        IFID_PCplus2 <= PCplus2;
        IFID_IR <= IMemory[PC>>1];
    // ID
        IDEX_IR <= IFID_IR; // For monitoring the pipeline
        // RegDst,ALUSrc,MemtoReg,RegWrite,MemWrite,Beq,Bne,ALUctl
        {IDEX_RegDst,IDEX_ALUSrc,IDEX_MemtoReg,IDEX_RegWrite,IDEX_MemWrite,IDEX_Beq,IDEX_Bne,IDEX_ALUctl} <= control;   
        IDEX_PCplus2 <= IFID_PCplus2;
        IDEX_RD1 <= RD1; 
        IDEX_RD2 <= RD2;
        IDEX_SignExt <= SignExtend;
        IDEX_rt <= IFID_IR[9:8];
        IDEX_rd <= IFID_IR[7:6];
    // EXE
        EXMEM_IR <= IDEX_IR; // For monitoring the pipeline
        EXMEM_RegWrite <= IDEX_RegWrite;
        EXMEM_MemtoReg <= IDEX_MemtoReg;
        EXMEM_Beq   <= IDEX_Beq;
        EXMEM_Bne   <= IDEX_Bne;
        EXMEM_MemWrite <= IDEX_MemWrite;
        EXMEM_Target <= Target;
        EXMEM_Zero <= Zero;
        EXMEM_ALUOut <= ALUOut;
        EXMEM_RD2 <= IDEX_RD2;
        EXMEM_rd <= WR;
    // MEM
        MEMWB_IR <= EXMEM_IR; // For monitoring the pipeline
        MEMWB_RegWrite <= EXMEM_RegWrite;
        MEMWB_MemtoReg <= EXMEM_MemtoReg;
        MEMWB_MemOut <= MemOut;
        MEMWB_ALUOut <= EXMEM_ALUOut;
        MEMWB_rd <= EXMEM_rd;
    // WB
    // Register write happens on neg edge of the clock (if MEMWB_RegWrite is asserted)
    end
endmodule

module test ();
  reg clock;
  wire signed [15:0] PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD;
  CPU test_cpu(clock,PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);
  always #1 clock = ~clock;
  initial begin
    $display ("PC   IFID_IR  IDEX_IR  EXMEM_IR MEMWB_IR  WD");
    $monitor ("%3d    %b     %b     %b     %b     %2d",PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);
    clock = 1;
    #69 $finish;
  end
endmodule
