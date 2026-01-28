`timescale 1ns / 1ns

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
