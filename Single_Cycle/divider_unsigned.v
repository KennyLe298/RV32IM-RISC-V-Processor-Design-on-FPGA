`timescale 1ns / 1ns
module divider_unsigned (
    input  [31:0] dividend,
    input  [31:0] divisor,
    
    output [31:0] quotient,
    output [31:0] remainder
);

    wire [31:0] stage_remainder [32:0];
    wire [31:0] stage_dividend  [32:0];
    wire [31:0] stage_quotient  [32:0];

    assign stage_remainder[0] = 32'b0;
    assign stage_quotient[0]  = 32'b0;
    assign stage_dividend[0]  = dividend; 

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : div_stage_gen
            divu_1iter iter_inst (
                .remainder_in(stage_remainder[i]),
                .dividend_in (stage_dividend[i]),
                .quotient_in (stage_quotient[i]),
                .divisor     (divisor), 
                
                .remainder_out(stage_remainder[i+1]),
                .dividend_out (stage_dividend[i+1]),
                .quotient_out (stage_quotient[i+1])
            );
        end
    endgenerate

    assign quotient  = stage_quotient[32];
    assign remainder = stage_remainder[32];

endmodule

