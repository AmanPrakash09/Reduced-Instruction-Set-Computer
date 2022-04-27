module datapath(mdata, sximm8, /*PC,*/ sixteenbitPC,
                writenum, write, readnum,clk,
                loada, loadb, loadc, loads,
                asel, bsel, sximm5, vsel,
                ALUop,
                shift,
                N, // shows if out is negative
                V, // shows if out has overflow
                Z, // shows if out is zero
                datapath_out);

// comments for above I/O's:
// mdata and PC are not included in I/O's since they are assigned as zeros --> need to be wires
// N, V, Z, replaced Z_out from lab5 since status register needs to have three 1-bit outputs


input [15:0] mdata;
// assign mdata = 16'b0;

input [15:0] sximm8;

// old PC wire
// wire [7:0] PC;
// assign PC = 8'b00000000;

// new input sixteenbitPC
input [15:0] sixteenbitPC;

// input [15:0] C; -> is datapath_out


input [1:0] vsel; // 2 bits wide
wire [15:0] data_in; //wire that comes out of "9" and goes into "1"
output [15:0] datapath_out;
//wire [15:0] datapath_out_copy;
//assign datapath_out_copy = datapath_out;


// wire [15:0] data_in;                 ---> already declared
input [2:0] writenum, readnum;
input write, clk;
wire [15:0] data_out; //wire that comes out of "1" and goes into "3" and "4"

// wire [15:0] data_out;                ---> already declared
// input clk;                           ---> already declared
input loada;
wire [15:0] loada_out; //wire that comes out of "3" and goes into "6"

// wire [15:0] data_out;                ---> already declared
// input clk;                           ---> already declared
input loadb;
wire [15:0] loadb_out; //wire that comes out of "4" and goes into "8"

// wire [15:0] loadb_out;                ---> already declared
input [1:0] shift;
wire [15:0] sout; //wire that comes out of "8" and goes into "7"

// wire [15:0] sout;                ---> already declared
input [15:0] sximm5;
input bsel;
wire [15:0] Bin; //wire that comes out of "7" and goes into "2"

input asel;
wire [15:0] Ain; //wire that comes out of "6" and goes into "2"

// wire [15:0] Ain;                ---> already declared
// wire [15:0] Bin;                ---> already declared
input [1:0] ALUop;
wire [15:0] alu_out; //wire that comes out of "2" and goes into "5"
wire [2:0] Z_alu; //wire that comes out of "2" and goes into "10"

// wire [15:0] alu_out;                ---> already declared
// input clk;                          ---> already declared
input loadc;
//wire [15:0] loadc_out;

// wire Z;                ---> already declared
// input clk;             ---> already declared
input loads;
output N; // shows if out is negative
output V; // shows if out has overflow
output Z; // shows if out is zero


// output reg [15:0] datapath_out;

// always @(*) begin
//     datapath_out = loadc_out;
// end

/*_________________________________________________________________________________________________*/

// datapath_in acts as a, datapath_out acts as b, vsel acts as selector, and data_in acts as out   
Mux41 v(mdata, sximm8, sixteenbitPC, datapath_out, vsel, data_in);

// instantiates regfile module
regfile REGFILE(data_in,writenum,write,readnum,clk,data_out);


// pipeline registers act similar to the registers used in "1" 
// used to load the values from "1" into the respecitive pipeline register
register #(16) A(data_out, loada, clk, loada_out);
register #(16) B(data_out, loadb, clk, loadb_out);

// instantiates shifter module
shifter U1(loadb_out, shift, sout);

// first variable acts as a, second acts as b, third acts as selector, and fourth acts as out   
Mux21 b(sximm5, sout, bsel, Bin);
Mux21 a(16'b0000000000000000, loada_out, asel, Ain);

// instantiates the ALU module
ALU U2(Ain,Bin,ALUop,alu_out,Z_alu);

// pipeline registers act similar to the registers used in "1" 
// used to load the values from "2" into the pipeline register
register #(16) C(alu_out, loadc, clk, datapath_out);

// pipeline registers act similar to the registers used in "1" 
// used to load the values from "2" into the pipeline register

// same logic as module register
status_register sr_Z(Z_alu[0], loads, clk, Z); // Z is 3 bits ---> need to take specific bit for Z_z0
status_register sr_N(Z_alu[1], loads, clk, N); // Z is 3 bits ---> need to take specific bit for N_z1
status_register sr_V(Z_alu[2], loads, clk, V); // Z is 3 bits ---> need to take specific bit for V_z2

endmodule

// ***LAB5 CODE*** begin

// determines if either a or b will be set to out depending on selector
module Mux21(a, b, selector, out);
input [15:0] a, b;
input selector;
output reg [15:0] out;

always @(*) begin 
    case(selector)
    1'b1: out = a; // if selector is 1, out is a
    1'b0: out = b; // if selector is 0, out is b
    endcase
end
endmodule

// ***LAB5 CODE*** end

// NEW LAB6 code for MUX

module Mux41(a, b, c, d, selector, out);
input [15:0] a, b, c, d; // all are 16 bits wide
input [1:0] selector; // need 2 bits
output reg [15:0] out; 

always @(*) begin 
    case(selector)
    2'b00: out = a; // if selector is 0, out is a
    2'b01: out = b; // if selector is 1, out is b
    2'b10: out = c; // if selector is 2, out is c
    2'b11: out = d; // if selector is 3, out is d
    default: out = 16'bxxxxxxxxxxxxxxxx;
    
    endcase
end
endmodule



// similar to register module written in regfile.v but has 1 bit input and output // taken out as we have PARAMATERIZED

module status_register(Z_bit,loads,clk,out); //could have used parameters
input Z_bit;
input loads;
input clk;
output out;

wire next_state_out;

assign next_state_out = loads ? Z_bit : out; //if load is 1, then bit of Z , if loads is 0, then undefined
status_vDFF v0(clk, next_state_out, out); //is the flip-flop, the assign statement below is the MUX

endmodule

module status_vDFF(clk, in, out) ;
  input clk ;
  input in ;
  output out ;
  reg out ;

  always @(posedge clk)
    out = in ; //makes the next_state_out the out (present_state)
endmodule

// module Status(Z, loads, clk, N_z1, V_z2, Z_z0);

// input [2:0] Z;
// input loads, clk;
// output reg N_z1, V_z2, Z_z0;

// always @(posedge clk) begin
    
//     N_z1 = Z[1]; // negative 
//     V_z2 = Z[2]; // overflow
//     Z_z0 = Z[0]; // zero

// end

// endmodule