module datapath(
  input clk,
  input rstn,
  output reg [15:0] bus
);
  reg [2:0] DR;
  reg [2:0] SR1;
  wire [2:0] SR2;
  wire [15:0] ALUA;
  wire [15:0] ALUB;

  reg R;
  reg BEN;
  reg [15:0] IR;

  reg LD_BEN;
  reg LD_MAR;
  reg LD_MDR;
  reg LD_IR;
  reg LD_PC;
  reg LD_REG;
  reg LD_CC;
  reg GateMARMUX;
  reg GateMDR;
  reg GateALU;
  reg GatePC;
  reg MARMUX;
  reg [1:0] PCMUX;
  reg ADDR1MUX;
  reg [1:0] ADDR2MUX;
  reg [1:0] DRMUX;
  reg [1:0] SR1MUX;
  reg [1:0] ALUK;
  reg MIO_EN;
  reg R_W;

  reg [15:0] SR1OUT;
  reg [15:0] SR2OUT;
  reg [15:0] ALUout;
  reg [15:0] MARMUXout;
  reg [15:0] PCout;

  reg [15:0] MARout;
  reg [15:0] MDRout;

  reg [15:0] DRIN;
  

  regfile regfile1 (clk, rstn, DRIN, DR, SR1, SR2, LD_REG, SR1OUT, SR2OUT);

  alu alu1 (ALUA, ALUB, ALUK, ALUout);
  pc_mar_muxes pc_mar_muxes1 (clk, rstn, MARMUX, ADDR2MUX, ADDR1MUX, PCMUX, LD_PC, IR, SR1OUT, bus, MARMUXout, PCout);
  control_unit control_unit1 (clk, R, BEN, IR, LD_BEN, LD_MAR, LD_MDR, LD_IR, LD_PC, LD_REG, LD_CC, GateMARMUX, GateMDR,
  GateALU, GatePC, MARMUX, PCMUX, ADDR1MUX, ADDR2MUX, DRMUX, SR1MUX, ALUK, MIO_EN, R_W);
  BEN_CC BEN_CC1 (clk, rstn, bus, IR, LD_BEN, LD_CC, BEN);
  datapath_memory datapath_memory1 (clk, rstn, bus, IR, LD_MAR, LD_MDR, MIO_EN, R_W, R, MARout, MDRout);

  always @(*) begin
    case (DRMUX)
      2'b00: DR = IR[11:9];
      2'b01: DR = 3'b111;
      2'b10: DR = 3'b110;
      2'b11: DR = 3'bxxx;
    endcase
    case (SR1MUX)
      2'b00: SR1 = IR[11:9];
      2'b01: SR1 = IR[8:6];
      2'b10: SR1 = 3'b110;
      2'b11: SR1 = 3'bxxx;
    endcase
    case ({GatePC, GateMARMUX, GateALU, GateMDR})
       4'b1000: bus = PCout;
       4'b0100: bus = MARMUXout;
       4'b0010: bus = ALUout;
       4'b0001: bus = MDRout;
    endcase
  end

  assign DRIN = bus;
  assign SR2 = IR[2:0];
  assign ALUA = SR1OUT;
  assign ALUB = (IR[5]) ? {{27{IR[4]}}, IR[4:0]} : SR2OUT;
endmodule

module control_unit (
  input clk,
  input R,
  input BEN,
  inout reg [15:0] IR,

  output reg LD_BEN,
  output reg LD_MAR,
  output reg LD_MDR,
  output reg LD_IR,
  output reg LD_PC,
  output reg LD_REG,
  output reg LD_CC,
  output reg GateMARMUX,
  output reg GateMDR,
  output reg GateALU,
  output reg GatePC,
  output reg MARMUX,
  output reg [1:0] PCMUX,
  output reg ADDR1MUX,
  output reg [1:0] ADDR2MUX,
  output reg [1:0] DRMUX,
  output reg [1:0] SR1MUX,
  output reg [1:0] ALUK,
  output reg MIO_EN,
  output reg R_W
);

  typedef enum {ADD, AND, NOT, TRAP, TRAP_MEM, TRAP_SET_PC, LEA, LD, LD_MEM, LD_SET_DR, LDR, LDI, LDI_MEM, LDI_SET_MAR, ST, ST_SET_MDR, ST_MEM, STR, STI, STI_MEM, STI_SET_MAR,
  JSR_CHECK, JSR, JSRR, JMP, BR_CHECK, BR, FETCH, FETCH_MEM, FETCH_SET_IR, DECODE} microsequencer_state;

  microsequencer_state state;
  microsequencer_state state_next;

  always @(*) begin
    // Default Values for Control Signals.
    LD_BEN = 1'b0;
    LD_MAR = 1'b0;
    LD_MDR = 1'b0;
    LD_IR = 1'b0;
    LD_PC = 1'b0;
    LD_REG = 1'b0;
    LD_CC=  1'b0;
    GateMARMUX = 1'b0;
    GateMDR = 1'b0;
    GateALU = 1'b0;
    GatePC = 1'b0;
    MARMUX  = 1'bx;
    PCMUX = 2'bxx;
    ADDR1MUX = 1'bx;
    ADDR2MUX = 2'bxx;
    DRMUX = 2'bxx;
    SR1MUX = 2'bxx;
    ALUK = 2'bxx;
    MIO_EN = 1'b0;
    R_W = 1'bx;

    case (state)
      FETCH: begin
        state_next = FETCH_MEM;
        LD_MAR = 1'b1;
        GatePC = 1'b1;
        LD_PC = 1'b1;
        PCMUX = 2'b00;
      end
      FETCH_MEM: begin
        state_next = microsequencer_state'((R) ? FETCH_SET_IR : FETCH_MEM);
        LD_MDR = 1'b1;
        MIO_EN = 1'b1;
        R_W = 1'b0;
      end
      FETCH_SET_IR: begin
        state_next = DECODE;
        LD_IR = 1'b1;
        GateMDR = 1'b1;
      end
      DECODE: begin
        case (IR[15:12])
          4'd1: state_next = ADD;
          4'd5: state_next = AND;
          4'd9: state_next = NOT;
          4'd15: state_next = TRAP;
          4'd14: state_next = LEA;
          4'd2: state_next = LD;
          4'd6: state_next = LDR;
          4'd10: state_next = LDI;
          4'd11: state_next = STI;
          4'd7: state_next = STR;
          4'd3: state_next = ST;
          4'd4: state_next = JSR_CHECK;
          4'd12: state_next = JMP;
          4'd0: state_next = BR_CHECK;
        endcase
        LD_BEN = 1'b1;
      end
      ADD: begin
        state_next = FETCH;
        LD_REG = 1'b1;
        LD_CC = 1'b1;
        GateALU = 1'b1;
        DRMUX = 2'b00;
        SR1MUX = 2'b01;
        ALUK = 2'b00;
      end
      AND: begin
        state_next = FETCH;
        LD_REG = 1'b1;
        LD_CC = 1'b1;
        GateALU = 1'b1;
        DRMUX = 2'b00;
        SR1MUX = 2'b01;
        ALUK = 2'b01;
      end
      NOT: begin
        state_next = FETCH;
        LD_REG = 1'b1;
        LD_CC = 1'b1;
        GateALU = 1'b1;
        DRMUX = 2'b00;
        SR1MUX = 2'b01;
        ALUK = 2'b10;
      end
      TRAP: begin
        state_next = TRAP_MEM;
        LD_MAR = 1'b1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b0;
      end
      TRAP_MEM: begin
        state_next = microsequencer_state'((R) ? TRAP_SET_PC : TRAP_MEM);
        LD_MDR = 1'b1;
        MIO_EN = 1'b1;
        R_W = 1'b0;
        LD_REG = 1'b1;
        DRMUX = 2'b01;
        GatePC = 1'b1;
      end
      TRAP_SET_PC: begin
        state_next = FETCH;
        GateMDR = 1'b1;
        LD_PC = 1'b1;
        PCMUX = 2'b01;
      end
      LEA: begin
        state_next = FETCH;
        LD_REG = 1'b1;
        LD_CC = 1'b1;
        DRMUX = 2'b00;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b0;
        ADDR2MUX = 2'b10;
      end
      LD: begin
        state_next = LD_MEM;
        LD_MAR = 1'b1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b0;
        ADDR2MUX = 2'b10;
      end
      LD_MEM: begin
        state_next = microsequencer_state'((R) ? LD_SET_DR : LD_MEM);
        LD_MDR = 1'b1;
        MIO_EN = 1'b1;
        R_W = 1'b0;
      end
      LD_SET_DR: begin
        state_next = FETCH;
        LD_REG = 1'b1;
        DRMUX = 2'b00;
        GateMDR = 1'b1;
        LD_CC = 1'b1;
      end
      LDR: begin
        state_next = LD_MEM;
        LD_MAR = 1'b1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b1;
        ADDR2MUX = 2'b01;
        SR1MUX = 2'b01;
      end
      LDI: begin
        state_next = LDI_MEM;
        LD_MAR = 1'b1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b0;
        ADDR2MUX = 2'b10;
      end
      LDI_MEM: begin
        state_next = microsequencer_state'((R) ? LDI_SET_MAR : LDI_MEM);
        LD_MDR = 1'b1;
        MIO_EN = 1'b1;
        R_W = 1'b0;
      end
      LDI_SET_MAR: begin
        state_next = LD_MEM;
        LD_MAR = 1'b1;
        GateMDR = 1'b1;
      end
      ST: begin
        state_next = ST_SET_MDR;
        LD_MAR = 1'b1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b0;
        ADDR2MUX = 2'b10;
      end
      ST_SET_MDR: begin
        state_next = ST_MEM;
        LD_MDR = 1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b1;
        ADDR2MUX = 2'b00;
        SR1MUX = 2'b00;
      end
      ST_MEM: begin
        state_next = microsequencer_state'((R) ? FETCH : ST_MEM);
        MIO_EN = 1'b1;
        R_W = 1'b1;
      end
      STR: begin
        state_next = ST_SET_MDR;
        LD_MAR = 1'b1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b1;
        ADDR2MUX = 2'b01;
        SR1MUX = 2'b01;
      end
      STI: begin
        state_next = STI_MEM;
        LD_MAR = 1'b1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b0;
        ADDR2MUX = 2'b10;
      end
      STI_MEM: begin
        state_next = microsequencer_state'((R) ? STI_SET_MAR : STI_MEM);
        LD_MDR = 1'b1;
        MIO_EN = 1'b1;
        R_W = 1'b0;
      end
      STI_SET_MAR: begin
        state_next = ST_SET_MDR;
        LD_MAR = 1'b1;
        GateMDR = 1'b1;
      end
      JSR_CHECK: begin
        state_next = microsequencer_state'((IR[11]) ? JSR : JSRR);
        LD_REG = 1'b1;
        DRMUX = 2'b01;
        GatePC = 1'b1;
      end
      JSR: begin
        state_next = FETCH;
        LD_PC = 1'b1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b0;
        ADDR2MUX = 2'b11;
      end
      JSRR: begin
        state_next = FETCH;
        LD_PC = 1'b1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b1;
        ADDR2MUX = 2'b00;
        SR1MUX = 2'b01;
      end
      JMP: begin
        state_next = FETCH;
        LD_PC = 1'b1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b1;
        ADDR2MUX = 2'b00;
        SR1MUX = 2'b01;
      end
      BR_CHECK: begin
        state_next = microsequencer_state'((BEN) ? BR : FETCH);
      end
      BR: begin
        state_next = FETCH;
        LD_PC = 1'b1;
        GateMARMUX = 1'b1;
        MARMUX = 1'b1;
        ADDR1MUX = 1'b0;
        ADDR2MUX = 2'b10;
      end
    endcase
  end

  always @(posedge clk) begin
    state <= state_next;
  end
endmodule

module datapath_memory (
  input clk,
  input rstn,
  input [15:0] bus,
  input [15:0] IR,

  input LD_MAR,
  input LD_MDR,
  input MIO_EN,
  input R_W,

  output reg R,
  output reg [15:0] MAR,
  output reg [15:0] MDR
);

  reg [15:0] MEM [65535:0];

  reg [15:0] MEMout;

  always @(*) begin
    R = 1;
    if (rstn) begin
      MAR = 16'b0;
      MDR = 16'b0;
    end
    else begin
      if (LD_MAR) MAR = bus;
      if (LD_MDR) MDR = (MIO_EN) ? MEMout : bus;
      if (MIO_EN) begin
        R = 0;
        // Write
        if (R_W) MEM[MAR] = MDR;
        // Read
        else MEMout = MEM[MAR];
      end
    end
    R = 1;
  end
endmodule

module BEN_CC (
  input clk,
  input rstn,
  input [15:0] bus,
  input [15:0] IR,

  input reg LD_BEN,
  input reg LD_CC,

  output reg BEN
);

wire CC_N;
wire CC_Z;
wire CC_P;
wire [2:0] CC_calc;
reg [2:0] CC;
wire BEN_calc;

// CC Calculations
assign CC_N = bus[15];
assign CC_Z = ~| bus[15];
assign CC_P = ~(CC_N | CC_Z);
assign CC_calc = {CC_N, CC_Z, CC_P};

assign BEN_calc = (IR[11] & CC[2]) | (IR[10] & CC[1]) | (IR[9] & CC[0]);

always @(posedge clk) begin
  if (!rstn) begin
    CC <= 3'b0;
    BEN <= 1'b0;
  end
  else begin 
    if (LD_CC) CC <= CC_calc;
    if (LD_BEN) BEN <= BEN_calc;
  end
end

endmodule

module pc_mar_muxes (
  input clk,
  input rstn,
  input MARMUX,
  input [1:0] ADDR2MUX,
  input ADDR1MUX,
  input [1:0] PCMUX,
  input LD_PC,
  input [15:0] IR,
  input [15:0] SR1OUT,
  input [15:0] bus,
  output reg [15:0] MARMUXout,
  output [15:0] PCout
);
  reg [15:0] ADDER_ADDR1;
  reg [15:0] ADDER_ADDR2;
  wire [15:0] ADDER_OUT;
  reg [15:0] PC_IN;
  wire [15:0] PC;

  reg_16 PC_REG (clk, rstn, LD_PC, PC_IN, PC);

  assign ADDER_OUT = ADDER_ADDR1 + ADDER_ADDR2;
  assign PCout = PC;

  always @(*) begin
    case (ADDR1MUX)
      1'b0: ADDER_ADDR1 = PC;
      1'b1: ADDER_ADDR1 = SR1OUT;
    endcase
    case (ADDR2MUX)
      2'b00: ADDER_ADDR2 = 16'b0;
      2'b01: ADDER_ADDR2 = {{10{IR[5]}}, IR[5:0]};
      2'b10: ADDER_ADDR2 = {{7{IR[8]}}, IR[8:0]};
      2'b11: ADDER_ADDR2 = {{5{IR[10]}}, IR[10:0]};
    endcase
    case (PCMUX)
      2'b00: PC_IN = PC + 1;
      2'b01: PC_IN = bus;
      2'b10: PC_IN = ADDER_OUT;
    endcase
    case (MARMUX)
      1'b0: MARMUXout = {8'b0, IR[7:0]};
      1'b1: MARMUXout = ADDER_OUT;
    endcase
  end

endmodule


module alu(
  input [15:0] A,
  input [15:0] B,
  input [1:0] ALUK,
  output reg [15:0] out
);

  always @(*) begin
    case (ALUK)
      2'b00: out = A + B;
      2'b01: out = A & B;
      2'b10: out = ~A;
      2'b11: out = A;
    endcase
  end

endmodule

module regfile(
    input clk,
    input rstn,
    input [15:0] DRIN,
    input [2:0] DR,
    input [2:0] SR1,
    input [2:0] SR2,
    input LD,
    output reg [15:0] SR1OUT,
    output reg [15:0] SR2OUT
);
    reg [7:0] loads;
    reg [15:0] outs [7:0];
    
    reg_16 R0 (clk, rstn, loads[0], DRIN, outs[0]);
    reg_16 R1 (clk, rstn, loads[1], DRIN, outs[1]);
    reg_16 R2 (clk, rstn, loads[2], DRIN, outs[2]);
    reg_16 R3 (clk, rstn, loads[3], DRIN, outs[3]);
    reg_16 R4 (clk, rstn, loads[4], DRIN, outs[4]);
    reg_16 R5 (clk, rstn, loads[5], DRIN, outs[5]);
    reg_16 R6 (clk, rstn, loads[6], DRIN, outs[6]);
    reg_16 R7 (clk, rstn, loads[7], DRIN, outs[7]);
    
    always @(*) begin
        loads = 8'b0;
        if (LD) loads[DR] = 1;
        SR1OUT = outs[SR1];
        SR2OUT = outs[SR2];
    end
    
endmodule

module reg_16(
    input clk,
    input rstn,
    input load,
    input [15:0] wdata,
    output reg [15:0] out
);

    always @(posedge clk) begin
        if (!rstn) out <= 16'b0;
        else begin
            if (load) out <= wdata;
            else out <= out;
        end
    end

endmodule