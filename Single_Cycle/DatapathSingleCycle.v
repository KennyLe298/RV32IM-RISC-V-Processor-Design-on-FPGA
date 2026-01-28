`timescale 1ns / 1ns

`define REG_SIZE 31
`define OPCODE_SIZE 6

// Don't forget your previous ALUs
//`include "divider_unsigned.v"
//`include "cla.v"

module RegFile (
    input      [        4:0] rd,
    input      [`REG_SIZE:0] rd_data,
    input      [        4:0] rs1,
    output reg [`REG_SIZE:0] rs1_data,
    input      [        4:0] rs2,
    output reg [`REG_SIZE:0] rs2_data,
    input                    clk,
    input                    we,
    input                    rst
);
  localparam NumRegs = 32;
  
  reg [`REG_SIZE:0] regs[0:NumRegs-1];

  // TODO: your code here
  integer i;

  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < NumRegs; i = i + 1) begin
        regs[i] <= 32'b0;
      end
    end else begin
      if (we && (rd != 5'd0)) begin
        regs[rd] <= rd_data;    
      end
      regs[0] <= 32'b0;        
    end
  end
  always @(*) begin
    rs1_data = (rs1 == 5'd0) ? 32'b0 : regs[rs1];
    rs2_data = (rs2 == 5'd0) ? 32'b0 : regs[rs2];
  end
endmodule

module DatapathSingleCycle (
    input                    clk,
    input                    rst,
    output reg               halt,
    output     [`REG_SIZE:0] pc_to_imem,
    input      [`REG_SIZE:0] inst_from_imem,
    // addr_to_dmem is a read-write port
    output reg [`REG_SIZE:0] addr_to_dmem,
    input      [`REG_SIZE:0] load_data_from_dmem,
    output reg [`REG_SIZE:0] store_data_to_dmem,
    output reg [        3:0] store_we_to_dmem
);



  // components of the instruction
  wire [           6:0] inst_funct7;
  wire [           4:0] inst_rs2;
  wire [           4:0] inst_rs1;
  wire [           2:0] inst_funct3;
  wire [           4:0] inst_rd;
  wire [`OPCODE_SIZE:0] inst_opcode;

  assign {inst_funct7, inst_rs2, inst_rs1, inst_funct3, inst_rd, inst_opcode} = inst_from_imem;

  wire [11:0] imm_i;
  assign imm_i = inst_from_imem[31:20];
  wire [ 4:0] imm_shamt = inst_from_imem[24:20];

  // S - stores
  wire [11:0] imm_s;
  assign imm_s = {inst_funct7, inst_rd};

  // B - conditionals
  wire [12:0] imm_b;
  assign {imm_b[12], imm_b[10:1], imm_b[11], imm_b[0]} = {inst_funct7, inst_rd, 1'b0};

  // J - unconditional jumps
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {inst_from_imem[31:12], 1'b0};

  wire [`REG_SIZE:0] imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
  wire [`REG_SIZE:0] imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
  wire [`REG_SIZE:0] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  wire [`REG_SIZE:0] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};

  localparam [`OPCODE_SIZE:0] OpLoad    = 7'b00_000_11;
  localparam [`OPCODE_SIZE:0] OpStore   = 7'b01_000_11;
  localparam [`OPCODE_SIZE:0] OpBranch  = 7'b11_000_11;
  localparam [`OPCODE_SIZE:0] OpJalr    = 7'b11_001_11;
  localparam [`OPCODE_SIZE:0] OpMiscMem = 7'b00_011_11;
  localparam [`OPCODE_SIZE:0] OpJal     = 7'b11_011_11;

  localparam [`OPCODE_SIZE:0] OpRegImm  = 7'b00_100_11;
  localparam [`OPCODE_SIZE:0] OpRegReg  = 7'b01_100_11;
  localparam [`OPCODE_SIZE:0] OpEnviron = 7'b11_100_11;

  localparam [`OPCODE_SIZE:0] OpAuipc   = 7'b00_101_11;
  localparam [`OPCODE_SIZE:0] OpLui     = 7'b01_101_11;

  wire inst_lui    = (inst_opcode == OpLui    );
  wire inst_auipc  = (inst_opcode == OpAuipc  );
  wire inst_jal    = (inst_opcode == OpJal    );
  wire inst_jalr   = (inst_opcode == OpJalr   );

  wire inst_beq    = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b000);
  wire inst_bne    = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b001);
  wire inst_blt    = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b100);
  wire inst_bge    = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b101);
  wire inst_bltu   = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b110);
  wire inst_bgeu   = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b111);

  wire inst_lb     = (inst_opcode == OpLoad   ) & (inst_from_imem[14:12] == 3'b000);
  wire inst_lh     = (inst_opcode == OpLoad   ) & (inst_from_imem[14:12] == 3'b001);
  wire inst_lw     = (inst_opcode == OpLoad   ) & (inst_from_imem[14:12] == 3'b010);
  wire inst_lbu    = (inst_opcode == OpLoad   ) & (inst_from_imem[14:12] == 3'b100);
  wire inst_lhu    = (inst_opcode == OpLoad   ) & (inst_from_imem[14:12] == 3'b101);

  wire inst_sb     = (inst_opcode == OpStore  ) & (inst_from_imem[14:12] == 3'b000);
  wire inst_sh     = (inst_opcode == OpStore  ) & (inst_from_imem[14:12] == 3'b001);
  wire inst_sw     = (inst_opcode == OpStore  ) & (inst_from_imem[14:12] == 3'b010);

  wire inst_addi   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b000);
  wire inst_slti   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b010);
  wire inst_sltiu  = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b011);
  wire inst_xori   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b100);
  wire inst_ori    = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b110);
  wire inst_andi   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b111);

  wire inst_slli   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b001) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_srli   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b101) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_srai   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b101) & (inst_from_imem[31:25] == 7'b0100000);

  wire inst_add    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b000) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_sub    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b000) & (inst_from_imem[31:25] == 7'b0100000);
  wire inst_sll    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b001) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_slt    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b010) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_sltu   = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b011) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_xor    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b100) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_srl    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b101) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_sra    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b101) & (inst_from_imem[31:25] == 7'b0100000);
  wire inst_or     = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b110) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_and    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b111) & (inst_from_imem[31:25] == 7'd0      );

  wire inst_mul    = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b000    );
  wire inst_mulh   = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b001    );
  wire inst_mulhsu = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b010    );
  wire inst_mulhu  = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b011    );
  wire inst_div    = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b100    );
  wire inst_divu   = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b101    );
  wire inst_rem    = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b110    );
  wire inst_remu   = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b111    );

  wire inst_ecall  = (inst_opcode == OpEnviron) & (inst_from_imem[31:7] == 25'd0  );
  wire inst_fence  = (inst_opcode == OpMiscMem);

  // program counter
  reg [`REG_SIZE:0] pcNext, pcCurrent;
  always @(posedge clk) begin
    if (rst) begin
      pcCurrent <= 32'd0;
    end else begin
      pcCurrent <= pcNext;
    end
  end
  assign pc_to_imem = pcCurrent;
  
  // PC + 4
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

  // NOTE: don't rename your RegFile instance as the tests expect it to be `rf`
  // TODO: you will need to edit the port connections, however.
  wire [`REG_SIZE:0] rs1_data;
  wire [`REG_SIZE:0] rs2_data;
  reg                 reg_we;
  reg  [`REG_SIZE:0]  reg_write_data;

  RegFile rf (
    .clk      (clk),
    .rst      (rst),
    .we       (reg_we),
    .rd       (inst_rd),
    .rd_data  (reg_write_data),
    .rs1      (inst_rs1),
    .rs2      (inst_rs2),
    .rs1_data (rs1_data),
    .rs2_data (rs2_data)
  );

  wire [`REG_SIZE:0] divu_q;
  wire [`REG_SIZE:0] divu_r;

  divider_unsigned divu_core (
      .dividend(rs1_data),
      .divisor (rs2_data),
      .quotient(divu_q),
      .remainder(divu_r)
  );

  wire [`REG_SIZE:0] rs1_abs;
  wire [`REG_SIZE:0] rs2_abs;
  wire [`REG_SIZE:0] div_abs_q;
  wire [`REG_SIZE:0] div_abs_r;
  wire               rs_sign;

  assign rs_sign = rs1_data[31] ^ rs2_data[31];

  assign rs1_abs = rs1_data[31] ? (~rs1_data + 32'd1) : rs1_data;
  assign rs2_abs = rs2_data[31] ? (~rs2_data + 32'd1) : rs2_data;

  divider_unsigned div_signed_core (
      .dividend(rs1_abs),
      .divisor (rs2_abs),
      .quotient(div_abs_q),
      .remainder(div_abs_r)
  );

  wire [63:0] mul_ss = $signed({{32{rs1_data[31]}}, rs1_data}) * $signed({{32{rs2_data[31]}}, rs2_data});
  wire [63:0] mul_uu = {32'b0, rs1_data} * {32'b0, rs2_data};
  wire [63:0] mul_su = $signed({{32{rs1_data[31]}}, rs1_data}) * $signed({32'b0, rs2_data});  
  wire [`REG_SIZE:0] eff_addr_i = rs1_data + imm_i_sext;
  wire [`REG_SIZE:0] eff_addr_s = rs1_data + imm_s_sext;
   // Branch comparisons
  wire rs_eq  = (rs1_data == rs2_data);
  wire rs_lt  = ($signed(rs1_data) <  $signed(rs2_data));
  wire rs_ge  = ($signed(rs1_data) >= $signed(rs2_data));
  wire rs_ltu = (rs1_data <  rs2_data);
  wire rs_geu = (rs1_data >= rs2_data);

    // ALU and CLA
  reg  [`REG_SIZE:0] alu_src1;
  reg  [`REG_SIZE:0] alu_src2;
  reg  [`REG_SIZE:0] alu_result;
  wire [`REG_SIZE:0] cla_sum;

  // Branch control
  reg branch_taken;

  reg [7:0]  load_byte;
  reg [15:0] load_halfword;

  cla u_cla (
    .a   (alu_src1),
    .b   (alu_src2),
    .cin (1'b0),
    .sum (cla_sum)
  );

  wire [`REG_SIZE:0] imm_u = {inst_from_imem[31:12], 12'b0};

  reg illegal_inst;

  always @(*) begin
    illegal_inst        = 1'b0;
    halt                = 1'b0;

    pcNext              = pc_plus_4;
    branch_taken        = 1'b0;

    reg_we              = 1'b0;
    reg_write_data      = 32'b0;

    alu_src1            = rs1_data;
    alu_src2            = rs2_data;
    alu_result          = 32'b0;

    addr_to_dmem        = 32'b0;
    store_data_to_dmem  = 32'b0;
    store_we_to_dmem    = 4'b0000;
    
    load_byte      = 8'b0;
    load_halfword  = 16'b0;

    // halt
    if (inst_ecall) begin
      halt         = 1'b1;   
      illegal_inst = 1'b0;
    end

    // LUI: rd = imm_u 
    else if (inst_lui) begin
      reg_we         = 1'b1;
      reg_write_data = imm_u;
    end

    // ADDI: rd = rs1 + imm_i 
    else if (inst_addi) begin
      alu_src1       = rs1_data;
      alu_src2       = imm_i_sext;
      reg_we         = 1'b1;
      reg_write_data = cla_sum;
    end

    // R-type ALU ops 
    else if (inst_add || inst_sub || inst_and || inst_or || inst_xor ||
             inst_slt || inst_sltu || inst_sll || inst_srl || inst_sra) begin

      // Default ALU inputs
      alu_src1 = rs1_data;
      alu_src2 = rs2_data;

      // ADD / SUB 
      if (inst_add) begin
        // rs1 + rs2
        alu_result = cla_sum;
      end else if (inst_sub) begin
        // rs1 - rs2 = rs1 + (-rs2)
        alu_src2   = ~rs2_data + 32'd1; 
        alu_result = cla_sum;
      end

      // Bitwise
      else if (inst_and) begin
        alu_result = rs1_data & rs2_data;
      end else if (inst_or) begin
        alu_result = rs1_data | rs2_data;
      end else if (inst_xor) begin
        alu_result = rs1_data ^ rs2_data;
      end

      // Shifts 
      else if (inst_sll) begin
        alu_result = rs1_data << rs2_data[4:0];
      end else if (inst_srl) begin
        alu_result = rs1_data >> rs2_data[4:0];
      end else if (inst_sra) begin
        alu_result = $signed(rs1_data) >>> rs2_data[4:0];
      end

      // Set less than 
      else if (inst_slt) begin
        alu_result = $signed(rs1_data) < $signed(rs2_data) ? 32'd1 : 32'd0;
      end else if (inst_sltu) begin
        alu_result = (rs1_data < rs2_data) ? 32'd1 : 32'd0;
      end

      reg_we         = 1'b1;
      reg_write_data = alu_result;
    end

    //I-type ALU ops (slti, sltiu, xori, ori, andi, shifts)
    else if (inst_slti || inst_sltiu || inst_xori || inst_ori || inst_andi ||
             inst_slli || inst_srli || inst_srai) begin

      if (inst_slti) begin
        reg_write_data = ($signed(rs1_data) < $signed(imm_i_sext)) ? 32'd1 : 32'd0;
      end else if (inst_sltiu) begin
        reg_write_data = (rs1_data < imm_i_sext) ? 32'd1 : 32'd0;
      end else if (inst_xori) begin
        reg_write_data = rs1_data ^ imm_i_sext;
      end else if (inst_ori) begin
        reg_write_data = rs1_data | imm_i_sext;
      end else if (inst_andi) begin
        reg_write_data = rs1_data & imm_i_sext;
      end else if (inst_slli) begin
        reg_write_data = rs1_data << imm_shamt;
      end else if (inst_srli) begin
        reg_write_data = rs1_data >> imm_shamt;
      end else if (inst_srai) begin
        reg_write_data = $signed(rs1_data) >>> imm_shamt;
      end

      reg_we = 1'b1;
    end
    //AUIPC
    else if (inst_auipc) begin
      reg_we         = 1'b1;
      reg_write_data = pcCurrent + imm_u;
    end
    
    //JAL
    else if (inst_jal) begin
      reg_we         = 1'b1;
      reg_write_data = pc_plus_4;          

      pcNext         = pcCurrent + imm_j_sext;
    end
    
    //JALR
    else if (inst_jalr) begin
      reg_we         = 1'b1;
      reg_write_data = pc_plus_4;

      //  rs1 + imm_i
      alu_src1       = rs1_data;
      alu_src2       = imm_i_sext;
      pcNext         = (rs1_data + imm_i_sext) & ~32'b1;
    end
    
        // LOADS: lb, lh, lw, lbu, lhu
    else if (inst_lb || inst_lh || inst_lw || inst_lbu || inst_lhu) begin
      // Compute effective address 
      addr_to_dmem = eff_addr_i;

      case (eff_addr_i[1:0])
        2'b00: load_byte = load_data_from_dmem[7:0];
        2'b01: load_byte = load_data_from_dmem[15:8];
        2'b10: load_byte = load_data_from_dmem[23:16];
        2'b11: load_byte = load_data_from_dmem[31:24];
      endcase

      if (eff_addr_i[1] == 1'b0)
        load_halfword = load_data_from_dmem[15:0];
      else
        load_halfword = load_data_from_dmem[31:16];

      if (inst_lb) begin
        reg_write_data = {{24{load_byte[7]}}, load_byte};
      end
      else if (inst_lbu) begin
        reg_write_data = {24'b0, load_byte};
      end
      else if (inst_lh) begin
        reg_write_data = {{16{load_halfword[15]}}, load_halfword};
      end
      else if (inst_lhu) begin
        reg_write_data = {16'b0, load_halfword};
      end
      else begin // inst_lw
        reg_write_data = load_data_from_dmem;
      end

      reg_we = 1'b1;
    end


        // STORES: sb, sh, sw
    else if (inst_sb || inst_sh || inst_sw) begin
      // Effective address
      addr_to_dmem = eff_addr_s;
      store_data_to_dmem = 32'b0;
      store_we_to_dmem   = 4'b0000;

      if (inst_sb) begin
        // Store byte from rs2_data[7:0]
        case (eff_addr_s[1:0])
          2'b00: begin
            store_we_to_dmem        = 4'b0001;
            store_data_to_dmem[7:0] = rs2_data[7:0];
          end
          2'b01: begin
            store_we_to_dmem         = 4'b0010;
            store_data_to_dmem[15:8] = rs2_data[7:0];
          end
          2'b10: begin
            store_we_to_dmem          = 4'b0100;
            store_data_to_dmem[23:16] = rs2_data[7:0];
          end
          2'b11: begin
            store_we_to_dmem          = 4'b1000;
            store_data_to_dmem[31:24] = rs2_data[7:0];
          end
        endcase
      end
      else if (inst_sh) begin
        // Store halfword from rs2_data[15:0]
        if (eff_addr_s[1] == 1'b0) begin
          store_we_to_dmem          = 4'b0011;
          store_data_to_dmem[15:0]  = rs2_data[15:0];
        end else begin
          store_we_to_dmem          = 4'b1100;
          store_data_to_dmem[31:16] = rs2_data[15:0];
        end
      end
      else begin // inst_sw
        store_we_to_dmem   = 4'b1111;
        store_data_to_dmem = rs2_data;
      end

      reg_we = 1'b0;
    end

        // M-extension (MUL/DIV/REM)
        else if (inst_mul || inst_mulh || inst_mulhsu || inst_mulhu ||
             inst_div || inst_divu || inst_rem || inst_remu) begin
    
        reg_we = 1'b1;
    
        if (inst_mul) begin
            reg_write_data = mul_ss[31:0];
        end
        else if (inst_mulh) begin
            reg_write_data = mul_ss[63:32];
        end
        else if (inst_mulhsu) begin
            reg_write_data = mul_su[63:32];
        end
        else if (inst_mulhu) begin
            reg_write_data = mul_uu[63:32];
        end
    
        else if (inst_divu) begin
            reg_write_data = (rs2_data == 0) ? 32'hFFFFFFFF : divu_q;
        end
        else if (inst_remu) begin
            reg_write_data = (rs2_data == 0) ? rs1_data : divu_r;
        end
   
        else if (inst_div) begin
            if (rs2_data == 0) begin
                reg_write_data = 32'hFFFFFFFF;
            end
            else if (rs1_data == 32'h80000000 && rs2_data == 32'hFFFFFFFF) begin
                reg_write_data = 32'h80000000;
            end
            else begin
                if (rs_sign)
                    reg_write_data = ~div_abs_q + 32'd1;   
                else
                    reg_write_data = div_abs_q;
            end
        end
   
        else if (inst_rem) begin
            if (rs2_data == 0) begin
                reg_write_data = rs1_data;
            end
            else begin
                reg_write_data = rs1_data[31]
                                 ? (~div_abs_r + 32'd1)  
                                 : div_abs_r;
            end
        end
    end


    // Branches
    else if (inst_beq || inst_bne || inst_blt || inst_bge || inst_bltu || inst_bgeu) begin
      if (inst_beq)       branch_taken = rs_eq;
      else if (inst_bne)  branch_taken = ~rs_eq;
      else if (inst_blt)  branch_taken = rs_lt;
      else if (inst_bge)  branch_taken = rs_ge;
      else if (inst_bltu) branch_taken = rs_ltu;
      else if (inst_bgeu) branch_taken = rs_geu;

      pcNext = branch_taken ? (pcCurrent + imm_b_sext) : pc_plus_4;
    end

  
    else begin
      illegal_inst = 1'b1;
    end
  end


endmodule

/* A memory module that supports 1-cycle reads and writes, with one read-only port
 * and one read+write port.
 */
module MemorySingleCycle #(
    parameter NUM_WORDS = 512
) (
  input                    rst,                 // rst for both imem and dmem
  input                    clock_mem,           // clock for both imem and dmem
  input      [`REG_SIZE:0] pc_to_imem,          // must always be aligned to a 4B boundary
  output reg [`REG_SIZE:0] inst_from_imem,      // the value at memory location pc_to_imem
  input      [`REG_SIZE:0] addr_to_dmem,        // must always be aligned to a 4B boundary
  output reg [`REG_SIZE:0] load_data_from_dmem, // the value at memory location addr_to_dmem
  input      [`REG_SIZE:0] store_data_to_dmem,  // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
  // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
  // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
  input      [        3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  reg [`REG_SIZE:0] mem_array[0:NUM_WORDS-1];

  // preload instructions to mem_array
  initial begin
    $readmemh("mem_initial_contents.hex", mem_array);
  end

  localparam AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam AddrLsb = 2;

  always @(posedge clock_mem) begin
    inst_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
  end

  always @(negedge clock_mem) begin
   if (store_we_to_dmem[0]) begin
     mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
   end
   if (store_we_to_dmem[1]) begin
     mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
   end
   if (store_we_to_dmem[2]) begin
     mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
   end
   if (store_we_to_dmem[3]) begin
     mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
   end
   // dmem is "read-first": read returns value before the write
   load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
  end
endmodule

/*
This shows the relationship between clock_proc and clock_mem. The clock_mem is
phase-shifted 90° from clock_proc. You could think of one proc cycle being
broken down into 3 parts. During part 1 (which starts @posedge clock_proc)
the current PC is sent to the imem. In part 2 (starting @posedge clock_mem) we
read from imem. In part 3 (starting @negedge clock_mem) we read/write memory and
prepare register/PC updates, which occur at @posedge clock_proc.

        ____
 proc: |    |______
           ____
 mem:  ___|    |___
*/
module Processor (
    input  clock_proc,
    input  clock_mem,
    input  rst,
    output halt
);

  wire [`REG_SIZE:0] pc_to_imem, inst_from_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [        3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
    .rst                 (rst),
    .clock_mem           (clock_mem),
    // imem is read-only
    .pc_to_imem          (pc_to_imem),
    .inst_from_imem      (inst_from_imem),
    // dmem is read-write
    .addr_to_dmem        (mem_data_addr),
    .load_data_from_dmem (mem_data_loaded_value),
    .store_data_to_dmem  (mem_data_to_write),
    .store_we_to_dmem    (mem_data_we)
  );

  DatapathSingleCycle datapath (
    .clk                 (clock_proc),
    .rst                 (rst),
    .pc_to_imem          (pc_to_imem),
    .inst_from_imem      (inst_from_imem),
    .addr_to_dmem        (mem_data_addr),
    .store_data_to_dmem  (mem_data_to_write),
    .store_we_to_dmem    (mem_data_we),
    .load_data_from_dmem (mem_data_loaded_value),
    .halt                (halt)
  );

endmodule

