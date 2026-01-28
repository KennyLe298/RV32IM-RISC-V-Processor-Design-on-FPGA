`timescale 1ns / 1ps

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1(input wire a, b,
           output wire g, p);
   assign g = a & b;
   assign p = a | b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);

   // TODO: your code here 
   wire [3:0] gen_prefix;
   assign gen_prefix[0] = gin[0];
   assign gen_prefix[1] = gin[1] | (pin[1] & gen_prefix[0]);
   assign gen_prefix[2] = gin[2] | (pin[2] & gen_prefix[1]);
   assign gen_prefix[3] = gin[3] | (pin[3] & gen_prefix[2]);

   assign gout = gen_prefix[3];
   assign pout = &pin; 

   // compute actual carries for the low-order 3 bits
   // c0 = carry into bit1 = g0 | p0 * cin
   // c1 = carry into bit2 = g1 | p1*g0 | p1*p0*cin
   // c2 = carry into bit3 = g2 | p2*g1 | p2*p1*g0 | p2*p1*p0*cin
   wire c0, c1, c2;
   assign c0 = gin[0] | (pin[0] & cin);
   assign c1 = gin[1] | (pin[1] & gin[0]) | (pin[1] & pin[0] & cin);
   assign c2 = gin[2] | (pin[2] & gin[1]) | (pin[2] & pin[1] & gin[0]) | (pin[2] & pin[1] & pin[0] & cin);

   assign cout = {c2, c1, c0};

endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

   // TODO: your code here
   wire [7:0] gen_prefix;
   assign gen_prefix[0] = gin[0];
   genvar i;
   generate
      for (i = 1; i < 8; i = i + 1) begin : GEN_PREFIX
         assign gen_prefix[i] = gin[i] | (pin[i] & gen_prefix[i-1]);
      end
   endgenerate

   assign gout = gen_prefix[7];
   assign pout = &pin; 

   // compute carries 
   // c[0] = gin[0] | pin[0]*cin
   // c[i] = gin[i] | pin[i]*c[i-1]
   assign cout[0] = gin[0] | (pin[0] & cin);

   assign cout[1] = gin[1] | (pin[1] & gin[0]) | (pin[1] & pin[0] & cin);

   assign cout[2] = gin[2] | (pin[2] & gin[1]) | (pin[2] & pin[1] & gin[0]) | (pin[2] & pin[1] & pin[0] & cin);

   assign cout[3] = gin[3] | (pin[3] & gin[2]) | (pin[3] & pin[2] & gin[1]) | (pin[3] & pin[2] & pin[1] & gin[0]) | (pin[3] & pin[2] & pin[1] & pin[0] & cin);

   assign cout[4] = gin[4] | (pin[4] & gin[3]) | (pin[4] & pin[3] & gin[2]) | (pin[4] & pin[3] & pin[2] & gin[1]) | (pin[4] & pin[3] & pin[2] & pin[1] & gin[0]) | (pin[4] & pin[3] & pin[2] & pin[1] & pin[0] & cin);

   assign cout[5] = gin[5] | (pin[5] & gin[4]) | (pin[5] & pin[4] & gin[3]) | (pin[5] & pin[4] & pin[3] & gin[2]) | (pin[5] & pin[4] & pin[3] & pin[2] & gin[1]) | (pin[5] & pin[4] & pin[3] & pin[2] & pin[1] & gin[0]) | (pin[5] & pin[4] & pin[3] & pin[2] & pin[1] & pin[0] & cin);
   
   assign cout[6] = gin[6] | (pin[6] & gin[5]) | (pin[6] & pin[5] & gin[4]) | (pin[6] & pin[5] & pin[4] & gin[3]) | (pin[6] & pin[5] & pin[4] & pin[3] & gin[2]) | (pin[6] & pin[5] & pin[4] & pin[3] & pin[2] & gin[1]) | (pin[6] & pin[5] & pin[4] & pin[3] & pin[2] & pin[1] & gin[0]) | (pin[6] & pin[5] & pin[4] & pin[3] & pin[2] & pin[1] & pin[0] & cin);


endmodule

module cla
  (input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);

   // TODO: your code here
   wire [31:0] gin = a & b;
   wire [31:0] pin = a | b;

   wire [3:0] gout8;   
   wire [3:0] pout8;  
   wire [6:0] cout8_0, cout8_1, cout8_2, cout8_3; 
   wire cin_chunk0, cin_chunk1, cin_chunk2, cin_chunk3;

   // chunk 0 : bits [7:0]
   gp8 gp8_0(.gin(gin[7:0]),   .pin(pin[7:0]),   .cin(cin),          .gout(gout8[0]), .pout(pout8[0]), .cout(cout8_0));
   // chunk 1 : bits [15:8]
   gp8 gp8_1(.gin(gin[15:8]),  .pin(pin[15:8]),  .cin(cin_chunk1),   .gout(gout8[1]), .pout(pout8[1]), .cout(cout8_1));
   // chunk 2 : bits [23:16]
   gp8 gp8_2(.gin(gin[23:16]), .pin(pin[23:16]), .cin(cin_chunk2),   .gout(gout8[2]), .pout(pout8[2]), .cout(cout8_2));
   // chunk 3 : bits [31:24]
   gp8 gp8_3(.gin(gin[31:24]), .pin(pin[31:24]), .cin(cin_chunk3),   .gout(gout8[3]), .pout(pout8[3]), .cout(cout8_3));

   // compute carries 
   assign cin_chunk0 = cin;
   assign cin_chunk1 = gout8[0] | (pout8[0] & cin_chunk0);
   assign cin_chunk2 = gout8[1] | (pout8[1] & cin_chunk1);
   assign cin_chunk3 = gout8[2] | (pout8[2] & cin_chunk2);

   // Chunk 0 sums [7:0]
   wire [7:0] carry_in0;
   assign carry_in0[0] = cin_chunk0;
   assign carry_in0[1] = cout8_0[0];
   assign carry_in0[2] = cout8_0[1];
   assign carry_in0[3] = cout8_0[2];
   assign carry_in0[4] = cout8_0[3];
   assign carry_in0[5] = cout8_0[4];
   assign carry_in0[6] = cout8_0[5];
   assign carry_in0[7] = cout8_0[6];
   assign sum[7:0] = a[7:0] ^ b[7:0] ^ carry_in0;

   // Chunk 1 sums [15:8]
   wire [7:0] carry_in1;
   assign carry_in1[0] = cin_chunk1;
   assign carry_in1[1] = cout8_1[0];
   assign carry_in1[2] = cout8_1[1];
   assign carry_in1[3] = cout8_1[2];
   assign carry_in1[4] = cout8_1[3];
   assign carry_in1[5] = cout8_1[4];
   assign carry_in1[6] = cout8_1[5];
   assign carry_in1[7] = cout8_1[6];
   assign sum[15:8] = a[15:8] ^ b[15:8] ^ carry_in1;

   // Chunk 2 sums [23:16]
   wire [7:0] carry_in2;
   assign carry_in2[0] = cin_chunk2;
   assign carry_in2[1] = cout8_2[0];
   assign carry_in2[2] = cout8_2[1];
   assign carry_in2[3] = cout8_2[2];
   assign carry_in2[4] = cout8_2[3];
   assign carry_in2[5] = cout8_2[4];
   assign carry_in2[6] = cout8_2[5];
   assign carry_in2[7] = cout8_2[6];
   assign sum[23:16] = a[23:16] ^ b[23:16] ^ carry_in2;

   // Chunk 3 sums [31:24]
   wire [7:0] carry_in3;
   assign carry_in3[0] = cin_chunk3;
   assign carry_in3[1] = cout8_3[0];
   assign carry_in3[2] = cout8_3[1];
   assign carry_in3[3] = cout8_3[2];
   assign carry_in3[4] = cout8_3[3];
   assign carry_in3[5] = cout8_3[4];
   assign carry_in3[6] = cout8_3[5];
   assign carry_in3[7] = cout8_3[6];
   assign sum[31:24] = a[31:24] ^ b[31:24] ^ carry_in3;

endmodule
