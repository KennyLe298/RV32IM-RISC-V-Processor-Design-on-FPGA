`timescale 1ns / 1ns

`define REG_SIZE 31
`define OPCODE_SIZE 6

`include "cla.v"
`include "DividerUnsignedPipelined.v"

module RegFile (
  input      [4:0] rd,
  input      [`REG_SIZE:0] rd_data,
  input      [4:0] rs1,
  output reg [`REG_SIZE:0] rs1_data,
  input      [4:0] rs2,
  output reg [`REG_SIZE:0] rs2_data,
  input      clk,
  input      we,
  input      rst
);
  localparam NumRegs = 32;
  reg [`REG_SIZE:0] regs[0:NumRegs-1];
  integer i;
  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < NumRegs; i = i + 1) regs[i] <= 32'b0;
    end else begin
      if (we && (rd != 5'd0)) regs[rd] <= rd_data;
      regs[0] <= 32'b0;        
    end
  end
  always @(*) begin
    rs1_data = (rs1 == 5'd0) ? 32'b0 : regs[rs1];
    rs2_data = (rs2 == 5'd0) ? 32'b0 : regs[rs2];
  end
endmodule

module DatapathMultiCycle (
    input                    clk,
    input                    rst,
    output reg               halt,
    output     [`REG_SIZE:0] pc_to_imem,
    input      [`REG_SIZE:0] inst_from_imem,
    output reg [`REG_SIZE:0] addr_to_dmem,
    input      [`REG_SIZE:0] load_data_from_dmem,
    output reg [`REG_SIZE:0] store_data_to_dmem,
    output reg [        3:0] store_we_to_dmem
);
  wire [6:0] inst_funct7;
  wire [4:0] inst_rs2, inst_rs1, inst_rd;
  wire [2:0] inst_funct3;
  wire [`OPCODE_SIZE:0] inst_opcode;
  assign {inst_funct7, inst_rs2, inst_rs1, inst_funct3, inst_rd, inst_opcode} = inst_from_imem;

  wire [11:0] imm_i = inst_from_imem[31:20];
  wire [ 4:0] imm_shamt = inst_from_imem[24:20];
  wire [11:0] imm_s = {inst_funct7, inst_rd};
  wire [12:0] imm_b;
  assign {imm_b[12], imm_b[10:1], imm_b[11], imm_b[0]} = {inst_funct7, inst_rd, 1'b0};
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {inst_from_imem[31:12], 1'b0};

  wire [`REG_SIZE:0] imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
  wire [`REG_SIZE:0] imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
  wire [`REG_SIZE:0] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  wire [`REG_SIZE:0] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};
  wire [`REG_SIZE:0] imm_u      = {inst_from_imem[31:12], 12'b0};

  localparam [`OPCODE_SIZE:0] OpLoad=7'b00_000_11, OpStore=7'b01_000_11, OpBranch=7'b11_000_11, OpJalr=7'b11_001_11, OpJal=7'b11_011_11, OpRegImm=7'b00_100_11, OpRegReg=7'b01_100_11, OpEnviron=7'b11_100_11, OpAuipc=7'b00_101_11, OpLui=7'b01_101_11;

  wire inst_lui = (inst_opcode==OpLui);
  wire inst_auipc = (inst_opcode==OpAuipc);
  wire inst_jal = (inst_opcode==OpJal);
  wire inst_jalr = (inst_opcode==OpJalr);
  wire inst_beq = (inst_opcode==OpBranch) && (inst_funct3==3'b000);
  wire inst_bne = (inst_opcode==OpBranch) && (inst_funct3==3'b001);
  wire inst_blt = (inst_opcode==OpBranch) && (inst_funct3==3'b100);
  wire inst_bge = (inst_opcode==OpBranch) && (inst_funct3==3'b101);
  wire inst_bltu = (inst_opcode==OpBranch) && (inst_funct3==3'b110);
  wire inst_bgeu = (inst_opcode==OpBranch) && (inst_funct3==3'b111);
  wire inst_lb = (inst_opcode==OpLoad) && (inst_funct3==3'b000);
  wire inst_lh = (inst_opcode==OpLoad) && (inst_funct3==3'b001);
  wire inst_lw = (inst_opcode==OpLoad) && (inst_funct3==3'b010);
  wire inst_lbu = (inst_opcode==OpLoad) && (inst_funct3==3'b100);
  wire inst_lhu = (inst_opcode==OpLoad) && (inst_funct3==3'b101);
  wire inst_sb = (inst_opcode==OpStore) && (inst_funct3==3'b000);
  wire inst_sh = (inst_opcode==OpStore) && (inst_funct3==3'b001);
  wire inst_sw = (inst_opcode==OpStore) && (inst_funct3==3'b010);
  wire inst_addi = (inst_opcode==OpRegImm) && (inst_funct3==3'b000);
  wire inst_slti = (inst_opcode==OpRegImm) && (inst_funct3==3'b010);
  wire inst_sltiu = (inst_opcode==OpRegImm) && (inst_funct3==3'b011);
  wire inst_xori = (inst_opcode==OpRegImm) && (inst_funct3==3'b100);
  wire inst_ori = (inst_opcode==OpRegImm) && (inst_funct3==3'b110);
  wire inst_andi = (inst_opcode==OpRegImm) && (inst_funct3==3'b111);
  wire inst_slli = (inst_opcode==OpRegImm) && (inst_funct3==3'b001) && (inst_funct7==7'd0);
  wire inst_srli = (inst_opcode==OpRegImm) && (inst_funct3==3'b101) && (inst_funct7==7'd0);
  wire inst_srai = (inst_opcode==OpRegImm) && (inst_funct3==3'b101) && (inst_funct7==7'b0100000);
  wire inst_add = (inst_opcode==OpRegReg) && (inst_funct3==3'b000) && (inst_funct7==7'd0);
  wire inst_sub = (inst_opcode==OpRegReg) && (inst_funct3==3'b000) && (inst_funct7==7'b0100000);
  wire inst_sll = (inst_opcode==OpRegReg) && (inst_funct3==3'b001) && (inst_funct7==7'd0);
  wire inst_slt = (inst_opcode==OpRegReg) && (inst_funct3==3'b010) && (inst_funct7==7'd0);
  wire inst_sltu = (inst_opcode==OpRegReg) && (inst_funct3==3'b011) && (inst_funct7==7'd0);
  wire inst_xor = (inst_opcode==OpRegReg) && (inst_funct3==3'b100) && (inst_funct7==7'd0);
  wire inst_srl = (inst_opcode==OpRegReg) && (inst_funct3==3'b101) && (inst_funct7==7'd0);
  wire inst_sra = (inst_opcode==OpRegReg) && (inst_funct3==3'b101) && (inst_funct7==7'b0100000);
  wire inst_or = (inst_opcode==OpRegReg) && (inst_funct3==3'b110) && (inst_funct7==7'd0);
  wire inst_and = (inst_opcode==OpRegReg) && (inst_funct3==3'b111) && (inst_funct7==7'd0);
  
  wire is_m_ext = (inst_opcode==OpRegReg) && (inst_funct7==7'd1);
  wire inst_mul = is_m_ext && (inst_funct3==3'b000);
  wire inst_mulh = is_m_ext && (inst_funct3==3'b001);
  wire inst_mulhsu = is_m_ext && (inst_funct3==3'b010);
  wire inst_mulhu = is_m_ext && (inst_funct3==3'b011);
  wire inst_div = is_m_ext && (inst_funct3==3'b100);
  wire inst_divu = is_m_ext && (inst_funct3==3'b101);
  wire inst_rem = is_m_ext && (inst_funct3==3'b110);
  wire inst_remu = is_m_ext && (inst_funct3==3'b111);
  wire inst_ecall = (inst_opcode==OpEnviron) && (inst_from_imem[31:7]==25'd0);

  reg [`REG_SIZE:0] pcNext, pcCurrent;
  assign pc_to_imem = pcCurrent;
  wire [`REG_SIZE:0] pc_plus_4 = pcCurrent + 32'd4;

  reg [`REG_SIZE:0] cycles_current, num_inst_current;
  always @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
      num_inst_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
      if (!rst) begin
        num_inst_current <= num_inst_current + 1;
      end
    end
  end

  wire [`REG_SIZE:0] rs1_data, rs2_data;
  reg reg_we;
  reg [`REG_SIZE:0] reg_write_data;

  RegFile rf (.clk(clk), .rst(rst), .we(reg_we), .rd(inst_rd), .rd_data(reg_write_data), .rs1(inst_rs1), .rs2(inst_rs2), .rs1_data(rs1_data), .rs2_data(rs2_data));

  wire is_div_op = inst_div || inst_divu || inst_rem || inst_remu;
  reg [3:0] div_cycles; 
  wire stall;
  always @(posedge clk) begin
      if (rst) div_cycles <= 0;
      else if (is_div_op && div_cycles < 8) div_cycles <= div_cycles + 1;
      else div_cycles <= 0;
  end
  assign stall = is_div_op && (div_cycles < 8);

  always @(posedge clk) begin
    if (rst) pcCurrent <= 32'd0;
    else if (!stall) pcCurrent <= pcNext;
  end

  wire rs_sign = rs1_data[31] ^ rs2_data[31];
  wire [31:0] rs1_abs = rs1_data[31] ? (~rs1_data + 1) : rs1_data;
  wire [31:0] rs2_abs = rs2_data[31] ? (~rs2_data + 1) : rs2_data;
  wire [31:0] div_in_a = (inst_div || inst_rem) ? rs1_abs : rs1_data;
  wire [31:0] div_in_b = (inst_div || inst_rem) ? rs2_abs : rs2_data;
  wire [31:0] pipe_rem, pipe_quo;

  DividerUnsignedPipelined pipelined_divider (.clk(clk), .rst(rst), .stall(1'b0), .i_dividend(div_in_a), .i_divisor(div_in_b), .o_remainder(pipe_rem), .o_quotient(pipe_quo));

  wire [63:0] mul_ss = $signed({{32{rs1_data[31]}}, rs1_data}) * $signed({{32{rs2_data[31]}}, rs2_data});
  wire [63:0] mul_uu = {32'b0, rs1_data} * {32'b0, rs2_data};
  wire [63:0] mul_su = $signed({{32{rs1_data[31]}}, rs1_data}) * $signed({32'b0, rs2_data});

  wire [`REG_SIZE:0] eff_addr_i = rs1_data + imm_i_sext;
  wire [`REG_SIZE:0] eff_addr_s = rs1_data + imm_s_sext;

  reg [`REG_SIZE:0] alu_src1, alu_src2, alu_result;
  wire [`REG_SIZE:0] cla_sum;
  cla u_cla (.a(alu_src1), .b(alu_src2), .cin(1'b0), .sum(cla_sum));

  reg branch_taken;
  wire rs_eq = (rs1_data==rs2_data);
  wire rs_lt = ($signed(rs1_data)<$signed(rs2_data));
  wire rs_ge = ($signed(rs1_data)>=$signed(rs2_data));
  wire rs_ltu = (rs1_data<rs2_data);
  wire rs_geu = (rs1_data>=rs2_data);
  reg [7:0] load_byte;
  reg [15:0] load_halfword;
  reg illegal_inst;

  always @(*) begin
    illegal_inst=0; halt=0; pcNext=pc_plus_4; branch_taken=0; reg_we=0; reg_write_data=0;
    alu_src1=rs1_data; alu_src2=rs2_data; alu_result=0;
    addr_to_dmem=0; store_data_to_dmem=0; store_we_to_dmem=0; load_byte=0; load_halfword=0;

    if (stall) begin reg_we = 1'b0; end
    else begin
        if (inst_ecall) halt = 1'b1;
        else if (inst_lui) begin reg_we=1; reg_write_data=imm_u; end
        else if (inst_auipc) begin reg_we=1; reg_write_data=pcCurrent+imm_u; end
        else if (inst_jal) begin reg_we=1; reg_write_data=pc_plus_4; pcNext=pcCurrent+imm_j_sext; end
        else if (inst_jalr) begin reg_we=1; reg_write_data=pc_plus_4; alu_src1=rs1_data; alu_src2=imm_i_sext; pcNext=(rs1_data+imm_i_sext)&~32'b1; end
        else if (inst_beq || inst_bne || inst_blt || inst_bge || inst_bltu || inst_bgeu) begin
            if (inst_beq) branch_taken=rs_eq; else if (inst_bne) branch_taken=~rs_eq;
            else if (inst_blt) branch_taken=rs_lt; else if (inst_bge) branch_taken=rs_ge;
            else if (inst_bltu) branch_taken=rs_ltu; else if (inst_bgeu) branch_taken=rs_geu;
            pcNext = branch_taken ? (pcCurrent+imm_b_sext) : pc_plus_4;
        end
        else if (inst_lb||inst_lh||inst_lw||inst_lbu||inst_lhu) begin
            addr_to_dmem=eff_addr_i;
            case (eff_addr_i[1:0])
                2'b00: load_byte=load_data_from_dmem[7:0]; 2'b01: load_byte=load_data_from_dmem[15:8];
                2'b10: load_byte=load_data_from_dmem[23:16]; 2'b11: load_byte=load_data_from_dmem[31:24];
            endcase
            if (eff_addr_i[1]==0) load_halfword=load_data_from_dmem[15:0]; else load_halfword=load_data_from_dmem[31:16];
            if (inst_lb) reg_write_data={{24{load_byte[7]}},load_byte};
            else if (inst_lbu) reg_write_data={24'b0,load_byte};
            else if (inst_lh) reg_write_data={{16{load_halfword[15]}},load_halfword};
            else if (inst_lhu) reg_write_data={16'b0,load_halfword};
            else reg_write_data=load_data_from_dmem;
            reg_we=1;
        end
        else if (inst_sb||inst_sh||inst_sw) begin
            addr_to_dmem=eff_addr_s;
            if (inst_sb) case (eff_addr_s[1:0])
                    2'b00: begin store_we_to_dmem=1; store_data_to_dmem[7:0]=rs2_data[7:0]; end
                    2'b01: begin store_we_to_dmem=2; store_data_to_dmem[15:8]=rs2_data[7:0]; end
                    2'b10: begin store_we_to_dmem=4; store_data_to_dmem[23:16]=rs2_data[7:0]; end
                    2'b11: begin store_we_to_dmem=8; store_data_to_dmem[31:24]=rs2_data[7:0]; end
                endcase
            else if (inst_sh) if (eff_addr_s[1]==0) begin store_we_to_dmem=3; store_data_to_dmem[15:0]=rs2_data[15:0]; end
                              else begin store_we_to_dmem=12; store_data_to_dmem[31:16]=rs2_data[15:0]; end
            else begin store_we_to_dmem=15; store_data_to_dmem=rs2_data; end
        end
        else if (inst_addi||inst_add||inst_sub||inst_and||inst_or||inst_xor||inst_slt||inst_sltu||inst_sll||inst_srl||inst_sra||inst_slti||inst_sltiu||inst_andi||inst_ori||inst_xori||inst_slli||inst_srli||inst_srai) begin
            if (inst_addi) begin alu_src1=rs1_data; alu_src2=imm_i_sext; alu_result=cla_sum; end
            else if (inst_add) begin alu_src1=rs1_data; alu_src2=rs2_data; alu_result=cla_sum; end
            else if (inst_sub) begin alu_src1=rs1_data; alu_src2=~rs2_data+1; alu_result=cla_sum; end
            else if (inst_and) alu_result=rs1_data&rs2_data; else if (inst_or) alu_result=rs1_data|rs2_data;
            else if (inst_xor) alu_result=rs1_data^rs2_data; else if (inst_sll) alu_result=rs1_data<<rs2_data[4:0];
            else if (inst_srl) alu_result=rs1_data>>rs2_data[4:0]; else if (inst_sra) alu_result=$signed(rs1_data)>>>rs2_data[4:0];
            else if (inst_slt) alu_result=($signed(rs1_data)<$signed(rs2_data))?1:0;
            else if (inst_sltu) alu_result=(rs1_data<rs2_data)?1:0;
            else if (inst_slti) alu_result=($signed(rs1_data)<$signed(imm_i_sext))?1:0;
            else if (inst_sltiu) alu_result=(rs1_data<imm_i_sext)?1:0;
            else if (inst_xori) alu_result=rs1_data^imm_i_sext; else if (inst_ori) alu_result=rs1_data|imm_i_sext;
            else if (inst_andi) alu_result=rs1_data&imm_i_sext; else if (inst_slli) alu_result=rs1_data<<imm_shamt;
            else if (inst_srli) alu_result=rs1_data>>imm_shamt; else if (inst_srai) alu_result=$signed(rs1_data)>>>imm_shamt;
            reg_we=1; reg_write_data=alu_result;
        end
        else if (is_m_ext) begin
            reg_we=1;
            if (inst_mul) reg_write_data=mul_ss[31:0]; else if (inst_mulh) reg_write_data=mul_ss[63:32];
            else if (inst_mulhsu) reg_write_data=mul_su[63:32]; else if (inst_mulhu) reg_write_data=mul_uu[63:32];
            else if (inst_divu) reg_write_data=(rs2_data==0)?32'hFFFFFFFF:pipe_quo;
            else if (inst_remu) reg_write_data=(rs2_data==0)?rs1_data:pipe_rem;
            else if (inst_div) begin if (rs2_data==0) reg_write_data=32'hFFFFFFFF;
                else if (rs1_data==32'h80000000 && rs2_data==32'hFFFFFFFF) reg_write_data=32'h80000000;
                else reg_write_data=rs_sign?(~pipe_quo+1):pipe_quo; end
            else if (inst_rem) begin if (rs2_data==0) reg_write_data=rs1_data;
                else reg_write_data=rs1_data[31]?(~pipe_rem+1):pipe_rem; end
        end
        else illegal_inst=1;
    end
  end
endmodule

module MemorySingleCycle #(parameter NUM_WORDS = 512) (input rst, clock_mem, input [`REG_SIZE:0] pc_to_imem, output reg [`REG_SIZE:0] inst_from_imem, input [`REG_SIZE:0] addr_to_dmem, output reg [`REG_SIZE:0] load_data_from_dmem, input [`REG_SIZE:0] store_data_to_dmem, input [3:0] store_we_to_dmem);
  reg [`REG_SIZE:0] mem_array[0:NUM_WORDS-1];
  initial $readmemh("mem_initial_contents.hex", mem_array);
  localparam AddrMsb = $clog2(NUM_WORDS) + 1; localparam AddrLsb = 2;
  always @(posedge clock_mem) inst_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
  always @(negedge clock_mem) begin
    if (store_we_to_dmem[0]) mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
    if (store_we_to_dmem[1]) mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
    if (store_we_to_dmem[2]) mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
    if (store_we_to_dmem[3]) mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
    load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
  end
endmodule

module Processor (input clock_proc, clock_mem, rst, output halt);
  wire [`REG_SIZE:0] pc_to_imem, inst_from_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(.NUM_WORDS(8192)) memory (.rst(rst), .clock_mem(clock_mem), .pc_to_imem(pc_to_imem), .inst_from_imem(inst_from_imem), .addr_to_dmem(mem_data_addr), .load_data_from_dmem(mem_data_loaded_value), .store_data_to_dmem(mem_data_to_write), .store_we_to_dmem(mem_data_we));
  DatapathMultiCycle datapath (.clk(clock_proc), .rst(rst), .pc_to_imem(pc_to_imem), .inst_from_imem(inst_from_imem), .addr_to_dmem(mem_data_addr), .store_data_to_dmem(mem_data_to_write), .store_we_to_dmem(mem_data_we), .load_data_from_dmem(mem_data_loaded_value), .halt(halt));
endmodule
