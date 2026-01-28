`timescale 1ns / 1ns

module DividerUnsignedPipelined (
    input             clk, rst, stall,
    input      [31:0] i_dividend,
    input      [31:0] i_divisor,
    output reg [31:0] o_remainder,
    output reg [31:0] o_quotient
);
    reg [31:0] stage_dividend  [0:7];
    reg [31:0] stage_divisor   [0:7];
    reg [31:0] stage_remainder [0:7];
    reg [31:0] stage_quotient  [0:7];

    // --- STAGE 0 (Input Stage) ---
    // We handle the first stage explicitly to avoid indexing issues
    wire [31:0] s0_rem_0, s0_div_0, s0_quo_0;
    wire [31:0] s0_rem_1, s0_div_1, s0_quo_1;
    wire [31:0] s0_rem_2, s0_div_2, s0_quo_2;
    wire [31:0] s0_rem_3, s0_div_3, s0_quo_3;

    divu_1iter s0_iter0 (.remainder_in(32'b0),    .dividend_in(i_dividend), .quotient_in(32'b0),    .divisor(i_divisor), .remainder_out(s0_rem_0), .dividend_out(s0_div_0), .quotient_out(s0_quo_0));
    divu_1iter s0_iter1 (.remainder_in(s0_rem_0), .dividend_in(s0_div_0),   .quotient_in(s0_quo_0), .divisor(i_divisor), .remainder_out(s0_rem_1), .dividend_out(s0_div_1), .quotient_out(s0_quo_1));
    divu_1iter s0_iter2 (.remainder_in(s0_rem_1), .dividend_in(s0_div_1),   .quotient_in(s0_quo_1), .divisor(i_divisor), .remainder_out(s0_rem_2), .dividend_out(s0_div_2), .quotient_out(s0_quo_2));
    divu_1iter s0_iter3 (.remainder_in(s0_rem_2), .dividend_in(s0_div_2),   .quotient_in(s0_quo_2), .divisor(i_divisor), .remainder_out(s0_rem_3), .dividend_out(s0_div_3), .quotient_out(s0_quo_3));

    always @(posedge clk) begin
        if (rst) begin
            stage_dividend[0]  <= 0;
            stage_divisor[0]   <= 0;
            stage_remainder[0] <= 0;
            stage_quotient[0]  <= 0;
        end else begin
            stage_dividend[0]  <= s0_div_3;
            stage_remainder[0] <= s0_rem_3;
            stage_quotient[0]  <= s0_quo_3;
            stage_divisor[0]   <= i_divisor;
        end
    end

    genvar i;
    generate
        for (i = 1; i < 8; i = i + 1) begin : pipe_stages
            wire [31:0] rem_0, div_0, quo_0;
            wire [31:0] rem_1, div_1, quo_1;
            wire [31:0] rem_2, div_2, quo_2;
            wire [31:0] rem_3, div_3, quo_3;
            
            wire [31:0] prev_dividend  = stage_dividend[i-1];
            wire [31:0] prev_divisor   = stage_divisor[i-1];
            wire [31:0] prev_remainder = stage_remainder[i-1];
            wire [31:0] prev_quotient  = stage_quotient[i-1];

            divu_1iter iter0 (.remainder_in(prev_remainder), .dividend_in(prev_dividend), .quotient_in(prev_quotient), .divisor(prev_divisor), .remainder_out(rem_0), .dividend_out(div_0), .quotient_out(quo_0));
            divu_1iter iter1 (.remainder_in(rem_0), .dividend_in(div_0), .quotient_in(quo_0), .divisor(prev_divisor), .remainder_out(rem_1), .dividend_out(div_1), .quotient_out(quo_1));
            divu_1iter iter2 (.remainder_in(rem_1), .dividend_in(div_1), .quotient_in(quo_1), .divisor(prev_divisor), .remainder_out(rem_2), .dividend_out(div_2), .quotient_out(quo_2));
            divu_1iter iter3 (.remainder_in(rem_2), .dividend_in(div_2), .quotient_in(quo_2), .divisor(prev_divisor), .remainder_out(rem_3), .dividend_out(div_3), .quotient_out(quo_3));

            always @(posedge clk) begin
                if (rst) begin
                    stage_dividend[i]  <= 0;
                    stage_divisor[i]   <= 0;
                    stage_remainder[i] <= 0;
                    stage_quotient[i]  <= 0;
                end else begin
                    stage_dividend[i]  <= div_3;
                    stage_remainder[i] <= rem_3;
                    stage_quotient[i]  <= quo_3;
                    stage_divisor[i]   <= prev_divisor;
                end
            end
        end
    endgenerate

    always @(*) begin
        o_remainder = stage_remainder[7];
        o_quotient  = stage_quotient[7];
    end
endmodule

module divu_1iter (
    input      [31:0] remainder_in,
    input      [31:0] dividend_in,
    input      [31:0] quotient_in,
    input      [31:0] divisor,
    output     [31:0] remainder_out,
    output     [31:0] dividend_out,
    output     [31:0] quotient_out
);
    wire [31:0] next_remainder_shifted;
    assign next_remainder_shifted = (remainder_in << 1) | ((dividend_in >> 31) & 32'd1);
    
    wire comparison_result;
    assign comparison_result = (next_remainder_shifted >= divisor);
    
    assign quotient_out = (quotient_in << 1) | {31'b0, comparison_result};
    
    wire [31:0] divisor_neg = ~divisor + 32'd1;
    assign remainder_out = comparison_result ? (next_remainder_shifted + divisor_neg) : next_remainder_shifted;
    
    assign dividend_out = dividend_in << 1;
endmodule
