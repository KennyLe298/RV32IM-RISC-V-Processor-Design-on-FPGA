`timescale 1ns / 1ns

module tb_Processor;

  reg  clock_proc;
  reg  clock_mem;
  reg  rst;
  wire halt;

  Processor dut (
    .clock_proc (clock_proc),
    .clock_mem  (clock_mem),
    .rst        (rst),
    .halt       (halt)
  );

  initial begin
    $dumpfile("waveform_result.vcd");
    $dumpvars(0, tb_Processor);
  end

  localparam real PROC_PERIOD_NS = 10.0; // 100 MHz

  initial begin
    clock_proc = 1'b0;
    forever #(PROC_PERIOD_NS/2.0) clock_proc = ~clock_proc;
  end

  initial begin
    clock_mem = 1'b0;
    #(PROC_PERIOD_NS/4.0); 
    forever #(PROC_PERIOD_NS/2.0) clock_mem = ~clock_mem;
  end
  
  wire [31:0] pc       = dut.datapath.pcCurrent;
  wire [31:0] instr    = dut.datapath.inst_from_imem;
  wire [31:0] alu_out  = dut.datapath.alu_result;

  wire [31:0] cla_res  = dut.datapath.cla_sum;      
  wire [31:0] div_quo  = dut.datapath.divu_q;    
  wire [31:0] div_rem  = dut.datapath.divu_r;       
  
  wire        reg_we   = dut.datapath.reg_we;
  wire [4:0]  rd       = dut.datapath.inst_rd;
  wire [31:0] wdata    = dut.datapath.reg_write_data;

  integer cycle_count;

  initial begin
    rst = 1'b1;
    cycle_count = 0;
    
    $display("=============================================================================================================");
    $display(" Time | Cycle |    PC    |   Instr  |    ALU/CLA Result   |  Div Q / R  | Reg Write?");
    $display("=============================================================================================================");

    // Hold reset for a few cycles
    repeat (5) @(posedge clock_proc);
    rst = 1'b0;
  end

  always @(posedge clock_proc) begin
    if (!rst) begin
      cycle_count <= cycle_count + 1;
      if (cycle_count < 100) begin 
        $display("%5d | %5d | %h | %h | %h / %h | %h / %h | %s x%02d = %h", 
                 $time, 
                 cycle_count, 
                 pc, 
                 instr, 
                 alu_out, cla_res,   
                 div_quo, div_rem,    
                 (reg_we ? "WR" : "  "), rd, wdata
                 );
      end

      if (halt) begin
        $display("-------------------------------------------------------------------------------------------------------------");
        $display("[%0t] HALT Asserted. Execution finished at cycle %0d.", $time, cycle_count);
        $display("Final PC: %h", pc);
        $finish;
      end

      if (cycle_count >= 5000) begin
        $display("[%0t] TIMEOUT: Halt never asserted.", $time);
        $finish;
      end
    end
  end

endmodule