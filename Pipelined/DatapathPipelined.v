`timescale 1ns / 1ns

`define REG_SIZE 31
`define INST_SIZE 31
`define OPCODE_SIZE 6
`define DIVIDER_STAGES 8

//`include "cla.v"
//`include "DividerUnsignedPipelined.v"


module RegFile (
  input       [        4:0] rd,
  input       [`REG_SIZE:0] rd_data,
  input       [        4:0] rs1,
  output reg [`REG_SIZE:0] rs1_data,
  input       [        4:0] rs2,
  output reg [`REG_SIZE:0] rs2_data,
  input                     clk,
  input                     we,
  input                     rst
);
  localparam NumRegs = 32;
  reg [`REG_SIZE:0] regs[0:NumRegs-1];
  integer i;
  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < NumRegs; i = i + 1) regs[i] <= 32'd0;
    end else begin
      if (we && (rd != 0)) regs[rd] <= rd_data;
    end
  end

  always @(*) begin
    if (we && (rd == rs1) && (rd != 0)) rs1_data = rd_data;
    else rs1_data = (rs1 == 0) ? 32'd0 : regs[rs1];

    if (we && (rd == rs2) && (rd != 0)) rs2_data = rd_data;
    else rs2_data = (rs2 == 0) ? 32'd0 : regs[rs2];
  end
endmodule

module DatapathPipelined (
  input                     clk,
  input                     rst,
  output      [ `REG_SIZE:0] pc_to_imem,
  input       [`INST_SIZE:0] inst_from_imem,
  output wire  [ `REG_SIZE:0] addr_to_dmem,
  input       [ `REG_SIZE:0] load_data_from_dmem,
  output reg  [ `REG_SIZE:0] store_data_to_dmem,
  output reg  [         3:0] store_we_to_dmem,
  output reg                 halt,
  output reg  [ `REG_SIZE:0] trace_writeback_pc,
  output reg  [`INST_SIZE:0] trace_writeback_inst
);

  localparam [`OPCODE_SIZE:0] OpcodeLoad    = 7'b00_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeStore   = 7'b01_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeBranch  = 7'b11_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeJalr    = 7'b11_001_11;
  localparam [`OPCODE_SIZE:0] OpcodeJal     = 7'b11_011_11;
  localparam [`OPCODE_SIZE:0] OpcodeRegImm  = 7'b00_100_11;
  localparam [`OPCODE_SIZE:0] OpcodeRegReg  = 7'b01_100_11;
  localparam [`OPCODE_SIZE:0] OpcodeEnviron = 7'b11_100_11;
  localparam [`OPCODE_SIZE:0] OpcodeAuipc   = 7'b00_101_11;
  localparam [`OPCODE_SIZE:0] OpcodeLui     = 7'b01_101_11;

  reg [`REG_SIZE:0] cycles_current;
  always @(posedge clk) begin
    if (rst) cycles_current <= 0;
    else cycles_current <= cycles_current + 1;
  end

  reg [31:0] f_d_pc, f_d_inst;
  reg [31:0] d_x_pc, d_x_inst, d_x_rs1_data, d_x_rs2_data, d_x_imm;
  reg [4:0]  d_x_rs1_addr, d_x_rs2_addr, d_x_rd_addr;
  reg [6:0]  d_x_opcode, d_x_funct7;
  reg [2:0]  d_x_funct3;
  reg        d_x_is_branch, d_x_is_jal, d_x_is_jalr, d_x_is_auipc, d_x_is_lui;
  reg        d_x_is_load, d_x_is_store, d_x_is_alu_reg, d_x_is_alu_imm, d_x_reg_we, d_x_halt;
  
  reg [31:0] x_m_pc, x_m_inst, x_m_alu_result, x_m_rs2_data;
  reg [4:0]  x_m_rd_addr, x_m_rs2_addr;
  reg        x_m_reg_we, x_m_is_load, x_m_is_store, x_m_halt;
  reg [2:0]  x_m_funct3;
  
  reg [31:0] m_w_pc, m_w_inst, m_w_alu_result, m_w_mem_data;
  reg [4:0]  m_w_rd_addr;
  reg        m_w_reg_we, m_w_is_load, m_w_halt;


  wire stall_if, flush_id, stall_id, flush_ex, stall_div;

  // ===========================================================================
  // STAGE 1: FETCH
  // ===========================================================================
  reg  [31:0] pc_current;
  wire [31:0] pc_next;
  wire [31:0] pc_plus_4 = pc_current + 4;
  wire branch_taken;
  wire [31:0] branch_target;

  assign pc_next = (branch_taken) ? branch_target : pc_plus_4;
  assign pc_to_imem = pc_current;

  always @(posedge clk) begin
      if (rst) pc_current <= 32'd0;
      else if (!stall_if && !stall_div) pc_current <= pc_next;
  end

  always @(posedge clk) begin
      if (rst || flush_id) begin
          f_d_pc <= 0;
	  f_d_inst <= 0; 
      end else if (!stall_id && !stall_div) begin
          f_d_pc <= pc_current;
	  f_d_inst <= inst_from_imem;
      end
  end

  // ===========================================================================
  // STAGE 2: DECODE
  // ===========================================================================
  wire [31:0] inst = f_d_inst;
  wire [6:0]  opcode = inst[6:0];
  wire [4:0]  rd = inst[11:7], rs1 = inst[19:15], rs2 = inst[24:20];
  wire [2:0]  funct3 = inst[14:12];
  wire [6:0]  funct7 = inst[31:25];

  wire [31:0] imm_i = {{20{inst[31]}}, inst[31:20]};
  wire [31:0] imm_s = {{20{inst[31]}}, inst[31:25], inst[11:7]};
  wire [31:0] imm_b = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
  wire [31:0] imm_u = {inst[31:12], 12'b0};
  wire [31:0] imm_j = {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};

  wire is_lui = (opcode == OpcodeLui);
  wire is_auipc = (opcode == OpcodeAuipc);
  wire is_jal = (opcode == OpcodeJal);
  wire is_jalr = (opcode == OpcodeJalr);
  wire is_branch = (opcode == OpcodeBranch);
  wire is_load = (opcode == OpcodeLoad);
  wire is_store = (opcode == OpcodeStore);
  wire is_alu_i = (opcode == OpcodeRegImm);
  wire is_alu_r = (opcode == OpcodeRegReg);
  wire is_ecall = (opcode == OpcodeEnviron) && (funct3 == 0) && (inst[31:20] == 12'd0);
  
  wire reg_we_decode = (is_lui || is_auipc || is_jal || is_jalr || is_load || is_alu_i || is_alu_r);

  reg [31:0] imm_sel;
  always @(*) begin
      if (is_store) imm_sel = imm_s;
      else if (is_branch) imm_sel = imm_b;
      else if (is_lui || is_auipc) imm_sel = imm_u;
      else if (is_jal) imm_sel = imm_j;
      else imm_sel = imm_i; 
  end

  wire [31:0] wb_data, rs1_data_raw, rs2_data_raw;
  RegFile rf (.clk(clk), .rst(rst), .we(m_w_reg_we), .rd(m_w_rd_addr), .rd_data(wb_data), 
              .rs1(rs1), .rs1_data(rs1_data_raw), .rs2(rs2), .rs2_data(rs2_data_raw));

  always @(posedge clk) begin
    if (rst || flush_id) begin
        d_x_pc         <= 32'd0;
        d_x_inst       <= 32'd0;

        d_x_rs1_data   <= 32'd0;
        d_x_rs2_data   <= 32'd0;
        d_x_imm        <= 32'd0;

        d_x_rs1_addr   <= 5'd0;
        d_x_rs2_addr   <= 5'd0;
        d_x_rd_addr    <= 5'd0;

        d_x_opcode     <= 7'd0;
        d_x_funct3     <= 3'd0;
        d_x_funct7     <= 7'd0;

        d_x_is_branch  <= 1'b0;
        d_x_is_jal     <= 1'b0;
        d_x_is_jalr    <= 1'b0;
        d_x_is_auipc   <= 1'b0;
        d_x_is_lui     <= 1'b0;

        d_x_is_load    <= 1'b0;
        d_x_is_store   <= 1'b0;
        d_x_is_alu_reg <= 1'b0;
        d_x_is_alu_imm <= 1'b0;

        d_x_reg_we     <= 1'b0;
        d_x_halt       <= 1'b0;
    end else if (!stall_id) begin
        d_x_pc       <= f_d_pc;
        d_x_inst     <= f_d_inst;

        d_x_rs1_data <= rs1_data_raw;
        d_x_rs2_data <= rs2_data_raw;
        d_x_imm      <= imm_sel;

        d_x_rs1_addr <= rs1;
        d_x_rs2_addr <= rs2;
        d_x_rd_addr  <= rd;

        d_x_opcode   <= opcode;
        d_x_funct3   <= funct3;
        d_x_funct7   <= funct7;

        d_x_is_branch<= is_branch;
        d_x_is_jal   <= is_jal;
        d_x_is_jalr  <= is_jalr;
        d_x_is_auipc <= is_auipc;
        d_x_is_lui   <= is_lui;

        d_x_is_load  <= is_load;
        d_x_is_store <= is_store;
        d_x_is_alu_reg <= is_alu_r;
        d_x_is_alu_imm <= is_alu_i;

        d_x_reg_we   <= reg_we_decode;
        d_x_halt     <= is_ecall;
    end
end


  // ===========================================================================
  // STAGE 3: EXECUTE
  // ===========================================================================
  wire [1:0] forward_a_sel = (x_m_reg_we && (x_m_rd_addr != 0) && (x_m_rd_addr == d_x_rs1_addr)) ? 2'b10 :
                             (m_w_reg_we && (m_w_rd_addr != 0) && (m_w_rd_addr == d_x_rs1_addr)) ? 2'b01 : 2'b00;
  wire [1:0] forward_b_sel = (x_m_reg_we && (x_m_rd_addr != 0) && (x_m_rd_addr == d_x_rs2_addr)) ? 2'b10 :
                             (m_w_reg_we && (m_w_rd_addr != 0) && (m_w_rd_addr == d_x_rs2_addr)) ? 2'b01 : 2'b00;

  reg [31:0] alu_in_a_fwd, alu_in_b_fwd;
  always @(*) begin
      case (forward_a_sel)
          2'b00: alu_in_a_fwd = d_x_rs1_data;
          2'b10: alu_in_a_fwd = x_m_alu_result;
          2'b01: alu_in_a_fwd = wb_data;
          default: alu_in_a_fwd = d_x_rs1_data;
      endcase
      case (forward_b_sel)
          2'b00: alu_in_b_fwd = d_x_rs2_data;
          2'b10: alu_in_b_fwd = x_m_alu_result;
          2'b01: alu_in_b_fwd = wb_data;
          default: alu_in_b_fwd = d_x_rs2_data;
      endcase
  end

   wire [31:0] alu_src1 = alu_in_a_fwd;
   wire [31:0] alu_src2 =
       (d_x_is_alu_imm || d_x_is_load || d_x_is_store ||
        d_x_is_lui || d_x_is_auipc || d_x_is_jalr)
           ? d_x_imm
           : alu_in_b_fwd;

   wire [31:0] cla_sum_add;
   wire [31:0] cla_sum_sub;

   cla u_cla_add (
       .a  (alu_src1),
       .b  (alu_src2),
       .cin(1'b0),
       .sum(cla_sum_add)
   );

   // SUB: a - b = a + (~b + 1)
   cla u_cla_sub (
       .a  (alu_src1),
       .b  (~alu_src2),
       .cin(1'b1),
       .sum(cla_sum_sub)
   );

   wire m_ext_instr = d_x_is_alu_reg && (d_x_funct7 == 7'h01);

   wire m_is_mul    = m_ext_instr && (d_x_funct3 == 3'b000);
   wire m_is_mulh   = m_ext_instr && (d_x_funct3 == 3'b001);
   wire m_is_mulhsu = m_ext_instr && (d_x_funct3 == 3'b010);
   wire m_is_mulhu  = m_ext_instr && (d_x_funct3 == 3'b011);
   wire m_is_div    = m_ext_instr && (d_x_funct3 == 3'b100);
   wire m_is_divu   = m_ext_instr && (d_x_funct3 == 3'b101);
   wire m_is_rem    = m_ext_instr && (d_x_funct3 == 3'b110);
   wire m_is_remu   = m_ext_instr && (d_x_funct3 == 3'b111);

   // Multipliers
   wire [63:0] mul_ss = $signed(alu_src1) * $signed(alu_src2);           
   wire [63:0] mul_su = $signed(alu_src1) * $signed({1'b0, alu_src2});   
   wire [63:0] mul_uu = alu_src1 * alu_src2;                             

   localparam DIV_LAT = `DIVIDER_STAGES;
   reg [$clog2(DIV_LAT):0] div_cycles;

   wire [31:0] div_in_a = (m_is_div || m_is_rem) ?
                          (alu_src1[31] ? -alu_src1 : alu_src1) :
                          alu_src1;
   wire [31:0] div_in_b = (m_is_div || m_is_rem) ?
                          (alu_src2[31] ? -alu_src2 : alu_src2) :
                          alu_src2;

   wire [31:0] div_quo_u, div_rem_u;

   always @(posedge clk) begin
     if (rst) begin
       div_cycles <= 0;
     end else if ((m_is_div || m_is_divu || m_is_rem || m_is_remu) &&
                  (div_cycles < DIV_LAT - 1)) begin
       div_cycles <= div_cycles + 1;
     end else begin
       div_cycles <= 0;
     end
   end

   assign stall_div =
       (m_is_div || m_is_divu || m_is_rem || m_is_remu) &&
       (div_cycles < DIV_LAT - 1);

   DividerUnsignedPipelined div_u (
       .clk       (clk),
       .rst       (rst),
       .stall     (stall_div),
       .i_dividend(div_in_a),
       .i_divisor (div_in_b),
       .o_remainder(div_rem_u),
       .o_quotient(div_quo_u)
   );

   wire [31:0] div_quo_s =
       (m_is_div && (alu_src1[31] ^ alu_src2[31])) ? -div_quo_u : div_quo_u;
   wire [31:0] div_rem_s =
       (m_is_rem &&  alu_src1[31])                 ? -div_rem_u : div_rem_u;

   reg [31:0] alu_result;

   always @(*) begin
       alu_result = 32'd0;

       if (d_x_is_lui) begin
           alu_result = d_x_imm;
       end else if (d_x_is_auipc) begin
           alu_result = d_x_pc + d_x_imm;
       end else if (d_x_is_jal || d_x_is_jalr) begin
           alu_result = d_x_pc + 4;
       end else if (d_x_is_load || d_x_is_store) begin
           alu_result = cla_sum_add;
       end else if (d_x_is_alu_reg || d_x_is_alu_imm) begin
           case (d_x_funct3)
             3'b000: begin
               if (d_x_is_alu_reg && d_x_funct7 == 7'h20) begin
                   alu_result = cla_sum_sub;
               end else if (m_is_mul) begin
                   alu_result = mul_ss[31:0];
               end else begin
                   alu_result = cla_sum_add;
               end
             end

             3'b001: begin
               if (m_is_mulh) begin
                   alu_result = mul_ss[63:32];
               end else begin
                   alu_result = alu_src1 << alu_src2[4:0];
               end
             end

             3'b010: begin
               if (m_is_mulhsu) begin
                   alu_result = mul_su[63:32];
               end else begin
                   alu_result = ($signed(alu_src1) < $signed(alu_src2)) ? 32'd1 : 32'd0;
               end
             end

             3'b011: begin
               if (m_is_mulhu) begin
                   alu_result = mul_uu[63:32];
               end else begin
                   alu_result = (alu_src1 < alu_src2) ? 32'd1 : 32'd0;
               end
             end

             3'b100: begin
               if (m_is_div) begin
                   alu_result = div_quo_s;     
               end else begin
                   alu_result = alu_src1 ^ alu_src2;
               end
             end

             3'b101: begin
               if (m_is_divu) begin
                   alu_result = div_quo_u;   
               end else if (d_x_funct7 == 7'h20) begin
                   alu_result = $signed(alu_src1) >>> alu_src2[4:0];
               end else begin
                   alu_result = alu_src1 >> alu_src2[4:0];
               end
             end

             3'b110: begin
               if (m_is_rem) begin
                   alu_result = div_rem_s;     
               end else begin
                   alu_result = alu_src1 | alu_src2;
               end
             end

             3'b111: begin
               if (m_is_remu) begin
                   alu_result = div_rem_u;     
               end else begin
                   alu_result = alu_src1 & alu_src2;
               end
             end

             default: begin
               alu_result = 32'd0;
             end
           endcase
       end
   end

  wire is_eq  = (alu_in_a_fwd == alu_in_b_fwd);
  wire is_lt  = ($signed(alu_in_a_fwd) < $signed(alu_in_b_fwd));
  wire is_ltu = (alu_in_a_fwd < alu_in_b_fwd);
  reg take_branch;
  always @(*) begin
      take_branch = 0;
      if (d_x_is_branch) begin
          case (d_x_funct3)
              3'b000: take_branch = is_eq;
              3'b001: take_branch = !is_eq;
              3'b100: take_branch = is_lt;
              3'b101: take_branch = !is_lt;
              3'b110: take_branch = is_ltu;
              3'b111: take_branch = !is_ltu;
              default: take_branch = 0;
          endcase
      end
  end

  assign branch_target = (d_x_is_jalr) ? ((alu_in_a_fwd + d_x_imm) & ~32'd1) : (d_x_pc + d_x_imm);
  assign branch_taken  = take_branch || d_x_is_jal || d_x_is_jalr;

     always @(posedge clk) begin
     if (rst) begin
         x_m_pc         <= 0;
         x_m_inst       <= 0;
         x_m_alu_result <= 0;
         x_m_rs2_data   <= 0;
         x_m_rd_addr    <= 0;
         x_m_rs2_addr   <= 0;
         x_m_reg_we     <= 0;
         x_m_is_load    <= 0;
         x_m_is_store   <= 0;
         x_m_halt       <= 0;
         x_m_funct3     <= 0;
     end else if (!stall_div) begin
         x_m_pc         <= d_x_pc; 
         x_m_inst       <= d_x_inst;
         x_m_alu_result <= alu_result;
         x_m_rs2_data   <= alu_in_b_fwd;
         x_m_rd_addr    <= d_x_rd_addr;
         x_m_rs2_addr   <= d_x_rs2_addr;
         x_m_reg_we     <= d_x_reg_we;
         x_m_is_load    <= d_x_is_load;
         x_m_is_store   <= d_x_is_store;
         x_m_halt       <= d_x_halt;
         x_m_funct3     <= d_x_funct3;
     end
     // else: stall_div == 1 → hold previous x_m_* values
   end

 
  // ===========================================================================
  // STAGE 4: MEMORY
  // ===========================================================================
  assign addr_to_dmem = x_m_alu_result;
   wire [31:0] store_data_fwd =
       (m_w_reg_we && (m_w_rd_addr != 0) && (m_w_rd_addr == x_m_rs2_addr))
           ? wb_data
           : x_m_rs2_data;

  always @(*) begin
      store_we_to_dmem = 0; store_data_to_dmem = 0;
      if (x_m_is_store) begin
          case (x_m_funct3)
              3'b000: begin // SB
                  store_we_to_dmem = 4'b0001 << x_m_alu_result[1:0];
                  store_data_to_dmem = store_data_fwd << (x_m_alu_result[1:0] * 8);
              end
              3'b001: begin // SH
                  store_we_to_dmem = 4'b0011 << (x_m_alu_result[1] * 2);
                  store_data_to_dmem = store_data_fwd << (x_m_alu_result[1] * 16);
              end
              3'b010: begin // SW
                  store_we_to_dmem = 4'b1111;
                  store_data_to_dmem = store_data_fwd;
              end
             default: begin store_we_to_dmem = 0; store_data_to_dmem = 0; end
          endcase
      end
  end

  reg [31:0] loaded_val;
  always @(*) begin
      loaded_val = load_data_from_dmem;
      case (x_m_funct3)
          3'b000: case(x_m_alu_result[1:0]) // LB
                  0: loaded_val = {{24{load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};
                  1: loaded_val = {{24{load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};
                  2: loaded_val = {{24{load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};
                  3: loaded_val = {{24{load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};
                  endcase
          3'b001: if(x_m_alu_result[1]) loaded_val = {{16{load_data_from_dmem[31]}}, load_data_from_dmem[31:16]};
                  else                  loaded_val = {{16{load_data_from_dmem[15]}}, load_data_from_dmem[15:0]};
          3'b010: loaded_val = load_data_from_dmem;
          3'b100: case(x_m_alu_result[1:0]) // LBU
                  0: loaded_val = {24'b0, load_data_from_dmem[7:0]};
                  1: loaded_val = {24'b0, load_data_from_dmem[15:8]};
                  2: loaded_val = {24'b0, load_data_from_dmem[23:16]};
                  3: loaded_val = {24'b0, load_data_from_dmem[31:24]};
                  endcase
          3'b101: if(x_m_alu_result[1]) loaded_val = {16'b0, load_data_from_dmem[31:16]};
                  else                  loaded_val = {16'b0, load_data_from_dmem[15:0]};
          default: loaded_val = load_data_from_dmem;
      endcase
  end

  always @(posedge clk) begin
      if (rst) begin
          m_w_pc <= 0; m_w_inst <= 0; 
          m_w_alu_result <= 0; m_w_mem_data <= 0; m_w_rd_addr <= 0;
          m_w_reg_we <= 0; m_w_is_load <= 0; m_w_halt <= 0;
      end else begin
          m_w_pc <= x_m_pc; m_w_inst <= x_m_inst; 
          m_w_alu_result <= x_m_alu_result; m_w_mem_data <= loaded_val;
          m_w_rd_addr <= x_m_rd_addr; m_w_reg_we <= x_m_reg_we; m_w_is_load <= x_m_is_load; m_w_halt <= x_m_halt;
      end
  end

  // ===========================================================================
  // STAGE 5: WRITEBACK
  // ===========================================================================
  assign wb_data = (m_w_is_load) ? m_w_mem_data : m_w_alu_result;
  always @(posedge clk) begin
    if (rst)
        halt <= 0;
    else
        halt <= m_w_halt;
end


  // ==========================================================================
// STAGE 5: WRITEBACK + TRACE
// ==========================================================================
 always @(posedge clk) begin
    if (rst) begin
        trace_writeback_pc   <= 32'd0;
        trace_writeback_inst <= 32'd0;
    end else begin
        trace_writeback_pc   <= m_w_pc;
        trace_writeback_inst <= m_w_inst;
    end
end



  wire load_use_hazard = (d_x_is_load) && (d_x_rd_addr != 0) &&
                        ((d_x_rd_addr == rs1) || (d_x_rd_addr == rs2));

  assign stall_if = load_use_hazard || stall_div;
  assign stall_id = load_use_hazard || stall_div;

  assign flush_id = branch_taken;
  assign flush_ex = branch_taken;

endmodule

module MemorySingleCycle #(
    parameter NUM_WORDS = 512
)(
    input  wire                     clk,
    input  wire                     rst,
    input  wire [31:0]              pc_to_imem,
    output reg  [31:0]              inst_from_imem,
    input  wire [31:0]              addr_to_dmem,
    input  wire [31:0]              store_data_to_dmem,
    input  wire [3:0]               store_we_to_dmem,
    output reg  [31:0]              load_data_from_dmem
);

    reg [31:0] mem_array [0:NUM_WORDS-1];

    initial begin
        $readmemh("mem_initial_contents.hex", mem_array);
    end

    localparam AddrLSB = 2;
    localparam AddrMSB = $clog2(NUM_WORDS) + 1;

    wire [AddrMSB-AddrLSB:0] imem_addr = pc_to_imem[AddrMSB:AddrLSB];
    wire [AddrMSB-AddrLSB:0] dmem_addr = addr_to_dmem[AddrMSB:AddrLSB];
    always @(posedge clk) begin
        inst_from_imem <= mem_array[imem_addr];
    end

    always @(posedge clk) begin
        if (store_we_to_dmem == 4'b1111) begin
            mem_array[dmem_addr] <= store_data_to_dmem;
        end
        load_data_from_dmem <= mem_array[dmem_addr];
    end

endmodule


module Processor (
    input clk, input rst, output halt,
    output [ `REG_SIZE:0] trace_writeback_pc, output [`INST_SIZE:0] trace_writeback_inst
);
    wire [`INST_SIZE:0] inst_from_imem;
    wire [ `REG_SIZE:0] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
    wire [          3:0] mem_data_we;
    wire [(8*32)-1:0] test_case;

    MemorySingleCycle #(.NUM_WORDS(8192)) memory (
        .rst(rst), .clk(clk),
        .pc_to_imem(pc_to_imem), .inst_from_imem(inst_from_imem),
        .addr_to_dmem(mem_data_addr), .load_data_from_dmem(mem_data_loaded_value),
        .store_data_to_dmem(mem_data_to_write), .store_we_to_dmem(mem_data_we)
    );

    DatapathPipelined datapath (
        .clk(clk), .rst(rst),
        .pc_to_imem(pc_to_imem), .inst_from_imem(inst_from_imem),
        .addr_to_dmem(mem_data_addr), .store_data_to_dmem(mem_data_to_write),
        .store_we_to_dmem(mem_data_we), .load_data_from_dmem(mem_data_loaded_value),
        .halt(halt),
        .trace_writeback_pc(trace_writeback_pc), .trace_writeback_inst(trace_writeback_inst)
    );
endmodule
