`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------
// single-cycle MIPS processor
// All of flip flops is implemented by Suyeong An


module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        memread, signext, shiftl16, memtoreg;
  wire        pcsrc, zeroE;
  wire        alusrc, regdst, regwrite, jump;
  wire [3:0]  alucontrol;
  wire        memwriteD;
  wire [31:0] instrD;
  
  controller c(
    .op         (instrD[31:26]), 
		.funct      (instrD[5:0]), 
		.zero       (zeroE),
		.signext    (signext),
    .clk (clk),
    .reset (reset),
    .memread    (memread),
		.shiftl16   (shiftl16),
		.memtoreg   (memtoreg),
		.memwrite   (memwriteD),
		.pcsrc      (pcsrc),
		.alusrc     (alusrc),
		.regdst     (regdst),
		.regwrite   (regwrite),
		.jump       (jump),
		.alucontrol (alucontrol));

		
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signextD    (signext),
    .memreadD    (memread),
    .shiftl16D   (shiftl16),
    .memtoregD   (memtoreg),
    .memwriteD   (memwriteD),
    .memwriteM   (memwrite),
    .pcsrcE      (pcsrc),
    .alusrcD     (alusrc),
    .regdstD     (regdst),
    .regwriteD   (regwrite),
    .jumpD       (jump),
    .alucontrolD (alucontrol),
    .zeroE       (zeroE),
    .pcF         (pc),
    .instrF      (instr),
    .instrD      (instrD),
    .aluoutM     (memaddr), 
    .writedataM  (memwritedata),
    .readdataM   (memreaddata));

endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  input       clk, reset,
                  output       memread,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regwrite,
                  output       jump,
                  output [3:0] alucontrol);

  wire [2:0] aluop;
  wire       branchD, branchE;
  reg        zero2;
  wire [5:0] opE;

  maindec md(
    .op       (op),
    .signext  (signext),
    .memread  (memread),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branchD),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
    .aluop    (aluop));

  aludec ad( 
    .funct      (funct),
    .aluop      (aluop), 
    .alucontrol (alucontrol));

flopr #(6) opIDtoEXff(clk, reset, op,opE);
flopr branchIDtoExff(clk, reset, branchD, branchE);
  always @(*) begin
    if (opE == 6'b000101)
	   zero2 <= !zero;
	 else 
	   zero2 <= zero;
  end
  
  assign pcsrc = branchE & zero2;
  

endmodule


module maindec(input  [5:0] op,
               output       signext, memread,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump,
               output [2:0] aluop);

  reg [12:0] controls;

  assign {memread, signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, aluop} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 13'b0001100000011; // Rtype
      6'b100011: controls <= #`mydelay 13'b1101010010000; // LW
      6'b101011: controls <= #`mydelay 13'b0100010100000; // SW
      6'b000100: controls <= #`mydelay 13'b0100001000001; // BEQ
		6'b000101: controls <= #`mydelay 13'b0100001000001; // BNE
      6'b001000, 
      6'b001001: controls <= #`mydelay 13'b0101010000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 13'b0001010000010; // ORI
      6'b001111: controls <= #`mydelay 13'b0011010000000; // LUI
      6'b000010: controls <= #`mydelay 13'b0000000001000; // J
		6'b000011: controls <= #`mydelay 13'b0001000001000; // JAL
		6'b001010: controls <= #`mydelay 13'b0101010000100; // SLTI
      default:   controls <= #`mydelay 13'b0000000000000; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [2:0] aluop,
              output reg [3:0] alucontrol);

  always @(*)
    case(aluop)
      3'b000: alucontrol <= #`mydelay 4'b0010;  // add
      3'b001: alucontrol <= #`mydelay 4'b1010;  // sub
      3'b010: alucontrol <= #`mydelay 4'b0001;  // or
		3'b100: alucontrol <= #`mydelay 4'b1011;  // SLTI
      default: case(funct)          // RTYPE
          6'b100000,
          6'b100001: alucontrol <= #`mydelay 4'b0010; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 4'b1010; // SUB, SUBU: only difference is exception
          6'b100100: alucontrol <= #`mydelay 4'b0000; // AND
          6'b100101: alucontrol <= #`mydelay 4'b0001; // OR
          6'b101010: alucontrol <= #`mydelay 4'b1011; // SLT
			 6'b101011: alucontrol <= #`mydelay 4'b1111; // SLTU we need to assign 4'b1111 
			 6'b001000: alucontrol <= #`mydelay 4'b0011; // jr
          default:   alucontrol <= #`mydelay 4'bxxxx; // ???
        endcase
    endcase
    
endmodule

module datapath(input         clk, reset,
                input         signextD, memreadD,
                input         memwriteD,
                input         shiftl16D,
                input         memtoregD, pcsrcE,
                input         alusrcD, regdstD,
                input         regwriteD, jumpD,
                input  [3:0]  alucontrolD,
                output        zeroE, memwriteM,
                output [31:0] pcF,
                input  [31:0] instrF,
                output [31:0] aluoutM, writedataM,
                output [31:0] instrD,
                input  [31:0] readdataM);


//###############HyunSeung Kim Start#########################
// ------------------------wire-----------------------------
// IF wire
  wire [31:0] pcplus4F, pcnextbrF,pcnext, pcjump;
  wire [31:0] instrFtmp;
  // ID wire
  wire [31:0] pcplus4D;
  wire [31:0] signimmD, shiftedimmD;
  wire [31:0] regsrcaD, srcaD, regsrcbD, writedataD;
  wire memreadDtmp, regwriteDtmp, jumpDtmp;
  wire memwriteDtmp;
  // EX wire
  wire memreadE, signextE, shiftl16E, memtoregE, alusrcE, regdstE, regwriteE, jumpE, memwriteE;
  wire [3:0] alucontrolE;
  wire [31:0] srcaE, srcbE, instrE, writedataE, srcaEpre, srcaEtmp, srcbEtmp,srcbEpre;
  wire [31:0] signimmE, signimmshE, shiftedimmE, aluoutE;
  wire [31:0] pcbranchE;
  wire [31:0] pcplus4E;
  wire [4:0] writeregtmpE, writeregE;
  // M wire
  wire memreadM, regwriteM, memtoregM, jumpM;
  wire [4:0] writeregM;
  wire [31:0] pcplus4M;
  // WB wire
  wire [31:0] resulttmpW, resultW, aluoutW, readdataW;
  wire [31:0] pcplus4W;
  wire [4:0] writeregW;
  wire regwriteW, memtoregW, jumpW;
  wire stall;  
  wire flush;

  reg pcsel;
  reg resultselW;
  //----------------forwarding unit----------------------------
  wire forwardaD, forwardbD;
  wire [1:0] forwardaE, forwardbE;
  forwarding_unit fu(
    .rsD (instrD[25:21]),
    .rtD (instrD[20:16]),
    .rsE (instrE[25:21]),
    .rtE (instrE[20:16]),
    .writeregM (writeregM),
    .writeregW (writeregW),
    .regwriteM (regwriteM),
    .regwriteW (regwriteW),
    .forwardaD (forwardaD),
    .forwardbD (forwardbD),
    .forwardaE (forwardaE),
    .forwardbE (forwardbE));
  //---------------hazard detection-----------------------------
  hazard_detector hd(
    .rtE (writeregE),
    .rtM (writeregM),
    .rsD (instrD[25:21]),
    .rtD (instrD[20:16]),
    .memreadE (memreadE),
    .memreadM (memreadM),
    .pcsrcE (pcsrcE),
    .jumpE (jumpE),
    .pcselE (pcsel),
    .stall (stall),
    .flush (flush)
  );

  
 //############HyunSeung Kim Finish####################
  
  
  // --------------------------logic-----------------------------
  // IF logic
  adder pcadd1(
    .a (pcF),
    .b (32'b100),
    .y (pcplus4F));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4F),
    .d1  (pcbranchE),
    .s   (pcsrcE),
    .y   (pcnextbrF));
  
  mux2 #(32) pcjmux(
    .d0   (pcnextbrF),
    .d1   ({pcplus4E[31:28], instrE[25:0], 2'b00}),
    .s    (jumpE),
    .y    (pcjump));
  
  always @(*) begin
    if (alucontrolE == 4'b0011)
	   pcsel <= 1;
	 else 
	   pcsel <= 0;
  end
  mux2 #(32) pcmux(
    .d0   (pcjump),
    .d1   (srcaE),
    .s    (pcsel),
    .y    (pcnext));
  
  flopenr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .en    (~stall),
    .d     (pcnext),
    .q     (pcF));
  mux2 #(32) instrm(
    .d0   (instrF),
    .d1   (32'b0),
    .s    (flush),
    .y    (instrFtmp));
   // ----------------------flip flop---------------------------------------
  // IF - ID
  flopenr #(32) instrFtoD(
    .clk (clk), 
    .reset (reset), 
    .en (~stall),
    .d (instrFtmp),
    .q (instrD));
  flopr #(32) pcplus4IFtoID(clk, reset, pcplus4F, pcplus4D);
   // ----------------------flip flop---------------------------------------
  // ID logic
  sign_zero_ext sze(
    .a       (instrD[15:0]),
    .signext (signextD),
    .y       (signimmD[31:0]));
  shift_left_16 sl16(
    .a         (signimmD[31:0]),
    .shiftl16  (shiftl16D),
    .y         (shiftedimmD[31:0]));
  regfile rf(
    .clk     (clk),
    .we      (regwriteW),
    .ra1     (instrD[25:21]),
    .ra2     (instrD[20:16]),
    .wa      (writeregW),
    .wd      (resultW),
    .rd1     (regsrcaD),
    .rd2     (regsrcbD));
//###############HyunSeung Kim Start#########################
  mux2 #(32) forwardingaID(
    .d0 (regsrcaD),
    .d1 (resultW),
    .s  (forwardaD),
    .y  (srcaD)
  );
  mux2 #(32) forwardingbID(
    .d0 (regsrcbD),
    .d1 (resultW),
    .s  (forwardbD),
    .y  (writedataD)
  );
  //############HyunSeung Kim Finish####################
  mux2 mw(
    .d0 (memwriteD),
    .d1 (1'b0),
    .s  (stall||flush),
    .y  (memwriteDtmp)
  );

  mux2 rw(
    .d0 (regwriteD),
    .d1 (1'b0),
    .s  (stall||flush),
    .y  (regwriteDtmp)
  );
  mux2 mr(
    .d0 (memreadD),
    .d1 (1'b0),
    .s  (stall||flush),
    .y  (memreadDtmp)
  );
  // mux2 jm(
  //   .d0 (jumpD),
  //   .d1 (1'b0),
  //   .s  (stall||flush),
  //   .y  (jumpDtmp),
  // );
 // ----------------------flip flop---------------------------------------
 // ID - EX flip flop
  flopr #(9) controlIDtoEXff(
  clk, reset, 
  {memreadDtmp, signextD, shiftl16D, memtoregD, alusrcD, regdstD, regwriteDtmp, jumpD, memwriteDtmp},
  {memreadE, signextE, shiftl16E, memtoregE, alusrcE, regdstE, regwriteE, jumpE, memwriteE} 
  );
  flopr #(32) instructionIDtoEXff(clk, reset, instrD, instrE);
  flopr #(32) signimmshIDtoEXff(clk, reset, signimmD, signimmE);
  flopr #(32) shiftedimmIDtoEXff(clk, reset, shiftedimmD, shiftedimmE);
  flopr #(4) alucontrolIDtoEXff(clk, reset, alucontrolD, alucontrolE);
  flopr #(32) writedataIDtoEXff(clk, reset, writedataD, writedataE);
  flopr #(32) srcaIDtoEXff(clk, reset, srcaD, srcaEpre);
  flopr #(32) pcplus4IDtoEX(clk, reset, pcplus4D, pcplus4E);
 // ----------------------flip flop---------------------------------------
  // EX logic
  mux2 #(5) wrmux(
    .d0  (instrE[20:16]),
    .d1  (instrE[15:11]),
    .s   (regdstE),
    .y   (writeregtmpE));
  mux2 #(5) wrmux2(    // jal instruction no need implementation
    .d0  (writeregtmpE),
    .d1  (5'b11111),
    .s   (jumpE),
    .y   (writeregE));

  mux2 #(32) srcbmux(
    .d0 (srcbEpre),
    .d1 (shiftedimmE),
    .s  (alusrcE),
    .y  (srcbE));
   alu alu(
    .a       (srcaE),
    .b       (srcbE),
    .alucont (alucontrolE),
    .result  (aluoutE),
    .zero    (zeroE));

  //###############HyunSeung Kim Start#########################
  mux2 #(32) forwardingaEX1(
    .d0 (srcaEpre),
    .d1 (aluoutM),
    .s  (forwardaE[0]),
    .y  (srcaEtmp)
  );
  mux2 #(32) forwardingaEX2(
    .d0 (srcaEtmp),
    .d1 (resultW),
    .s  (forwardaE[1]),
    .y  (srcaE)
  );
  mux2 #(32) forwardingbEX1(
    .d0 (writedataE),
    .d1 (aluoutM),
    .s  (forwardbE[0]),
    .y  (srcbEtmp)
  );
  mux2 #(32) forwardingbEX2(
    .d0 (srcbEtmp),
    .d1 (resultW),
    .s  (forwardbE[1]),
    .y  (srcbEpre)
  );

   sl2 immsh(
    .a (shiftedimmE),
    .y (signimmshE));
				 
  adder pcadd2(
    .a (pcplus4E),
    .b (signimmshE),
    .y (pcbranchE));
   
//############HyunSeung Kim Finish####################
  // ----------------------flip flop---------------------------------------
  // EX - M flip flop
  flopr #(32) aluoutEXtoMff(clk, reset, aluoutE, aluoutM);
  flopr #(5) writeregEXtoMff(clk, reset, writeregE, writeregM);
  flopr #(32) writedataEXtoMff(clk, reset, srcbEpre, writedataM);
  flopr #(5) controlEXtoMff(clk, reset, {jumpE, memwriteE, memreadE, regwriteE, memtoregE},{jumpM, memwriteM, memreadM, regwriteM, memtoregM});
  flopr #(32) pcplus4ExtoM(clk, reset, pcplus4E, pcplus4M);
  // ----------------------flip flop---------------------------------------
  // M logic
  
  
  // ----------------------flip flop---------------------------------------
  // M - WB flip flop
  flopr regwriteMtoWBff(clk, reset, regwriteM, regwriteW);
  flopr memtoregMtoWBff(clk, reset, memtoregM, memtoregW);
  flopr #(32) aluoutMtoWBff(clk, reset, aluoutM, aluoutW);
  flopr #(32) readdataMtoWBff(clk, reset, readdataM, readdataW);
  flopr #(5) writeregMtoWBff(clk, reset, writeregM, writeregW);
  flopr jumpMtoWBff(clk, reset, jumpM, jumpW);
  flopr #(32) pcplus4MxtoWB(clk, reset, pcplus4M, pcplus4W);
  // ----------------------flip flop---------------------------------------
  // WB logic
  
  always @(*) begin
    if (jumpW == 1 && regwriteW == 1)
	   resultselW <= 1;
	 else
	   resultselW <= 0;
  end

  mux2 #(32) resmux(
    .d0 (aluoutW),
    .d1 (readdataW),
    .s  (memtoregW),
    .y  (resulttmpW));

  mux2 #(32) resmux2(
    .d0 (resulttmpW),
	 .d1 (pcplus4W),
	 .s  (resultselW),
	 .y  (resultW));

endmodule

//###############HyunSeung Kim Start#########################

module forwarding_unit(
  input  [4:0] rsD, rsE, rtD, rtE, writeregM, writeregW,
  input  regwriteM, regwriteW,
  output reg forwardaD, forwardbD,
  output reg [1:0] forwardaE, forwardbE
);
  always @(*)
  begin
    forwardaD = 1'b0;
    forwardbD = 1'b0;
    forwardaE = 2'b00;
    forwardbE = 2'b00;
    if (rsD != 0)
      if((rsD == writeregW) && regwriteW)  forwardaD = 1'b1;
    if(rtD != 0)
      if((rtD == writeregW) && regwriteW)  forwardbD = 1'b1;
    
    if(rsE != 0)
    begin
      if((rsE == writeregM) && regwriteM)  forwardaE = 2'b01;
      else if((rsE == writeregW) && regwriteW)  forwardaE = 2'b10;
    end
    
    if(rtE != 0)
    begin
      if((rtE == writeregM) && regwriteM)  forwardbE = 2'b01;
      else if((rtE == writeregW) && regwriteW)  forwardbE = 2'b10;
    end
    
  end
endmodule

module hazard_detector(
  input [4:0] rtE, rtM, rsD, rtD,
  input memreadE, memreadM,
  input pcsrcE, jumpE, pcselE,
  output reg stall, flush
);
  always @(*)
  begin
    if(pcsrcE || jumpE || pcselE)  flush = 1;
    else  flush = 0;

    if((rtE == rsD || rtE == rtD) && memreadE)  stall = 1;
    else if((rtM == rsD || rtM == rtD) && memreadM)  stall = 1;
    else stall = 0;
  end
endmodule
//############HyunSeung Kim Finish####################