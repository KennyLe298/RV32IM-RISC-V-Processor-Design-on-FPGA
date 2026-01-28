`timescale 1ns / 1ps

module tb_Processor;
    reg clk;
    reg rst;

    wire halt;
    wire [31:0] trace_writeback_pc;
    wire [31:0] trace_writeback_inst;
    Processor dut (
        .clk(clk),
        .rst(rst),
        .halt(halt),
        .trace_writeback_pc(trace_writeback_pc),
        .trace_writeback_inst(trace_writeback_inst)
    );

    localparam real CLK_PERIOD = 10.0;

    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2.0) clk = ~clk;
    end

    initial begin
        $dumpfile("processor_waveform.vcd");
        $dumpvars(0, tb_Processor);
    end

    // FETCH STAGE
    wire [31:0] pc_f  = dut.datapath.pc_current;
    wire [31:0] inst_f = dut.datapath.inst_from_imem;

    // EXECUTE STAGE
    wire [31:0] alu_out = dut.datapath.alu_result;

    wire [31:0] div_quo_u = dut.datapath.div_quo_u;
    wire [31:0] div_rem_u = dut.datapath.div_rem_u;
    wire stall_div = dut.datapath.stall_div;
    wire [3:0] div_count = dut.datapath.div_cycles;

    // Register writeback info
    wire        reg_we = dut.datapath.m_w_reg_we;
    wire [4:0]  rd_w    = dut.datapath.m_w_rd_addr;
    wire [31:0] wb_data = dut.datapath.wb_data;

    // Pipeline PC and instruction
    wire [31:0] pc_d = dut.datapath.d_x_pc;
    wire [31:0] pc_x = dut.datapath.x_m_pc;
    wire [31:0] pc_w = dut.datapath.m_w_pc;

    integer cycle;

    initial begin
        rst = 1;
        cycle = 0;

        $display("====================================================================================");
        $display(" Time | Cycle |   PC(Fetch)   |  Instruction  | StallDIV | DivCnt |  ALU Result ");
        $display("====================================================================================");
        repeat(5) @(posedge clk);
        rst = 0;
    end

    always @(posedge clk) begin
        if (!rst) begin
            cycle <= cycle + 1;

            if (cycle < 500) begin
                $display("%5t | %5d | %h | %h |   %b     |   %2d   | %h",
                         $time,
                         cycle,
                         pc_f,
                         inst_f,
                         stall_div,
                         div_count,
                         alu_out);
            end

            // HALT handling
            if (halt) begin
                $display("------------------------------------------------------------------------------------");
                $display("[%0t] HALT asserted. Finished at cycle %0d.", $time, cycle);
                $display("Final WB PC   : %h", trace_writeback_pc);
                $display("Final WB INST : %h", trace_writeback_inst);
                $display("------------------------------------------------------------------------------------");
                $finish;
            end

            if (cycle >= 5000) begin
                $display("[%0t] TIMEOUT ERROR: Processor never halted.", $time);
                $finish;
            end
        end
    end

endmodule
