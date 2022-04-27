module regfile(data_in,writenum,write,readnum,clk,data_out);
input [15:0] data_in;
input [2:0] writenum, readnum;
input write, clk;
output [15:0] data_out;
// fill out the rest

wire [7:0]onehot_w; //wires coming out of 3:8 decoder for writenum

wire load7; //wires coming out of AND-gates (load becomes onehot depending on write)
wire load6;
wire load5;
wire load4;
wire load3;
wire load2;
wire load1;
wire load0;

wire [15:0]R0; //wires coming out of each register
wire [15:0]R1; 
wire [15:0]R2; 
wire [15:0]R3; 
wire [15:0]R4; 
wire [15:0]R5; 
wire [15:0]R6; 
wire [15:0]R7; 

wire [7:0]onehot_r; //wires coming out of 3:8 decoder for readnum

Dec38 w(writenum,onehot_w); //decoder for writenum

anded l0(onehot_w[0], write, load0); //anded wires after decoder
anded l1(onehot_w[1], write, load1);
anded l2(onehot_w[2], write, load2);
anded l3(onehot_w[3], write, load3);
anded l4(onehot_w[4], write, load4);
anded l5(onehot_w[5], write, load5);
anded l6(onehot_w[6], write, load6);
anded l7(onehot_w[7], write, load7);

register #(16) r0(data_in,load0,clk,R0); //register for R0
register #(16) r1(data_in,load1,clk,R1); //register for R1
register #(16) r2(data_in,load2,clk,R2); //register for R2
register #(16) r3(data_in,load3,clk,R3); //register for R3
register #(16) r4(data_in,load4,clk,R4); //register for R4
register #(16) r5(data_in,load5,clk,R5); //register for R5
register #(16) r6(data_in,load6,clk,R6); //register for R6
register #(16) r7(data_in,load7,clk,R7); //register for R7

Dec38 r(readnum,onehot_r); //decoder for readnum
Mux m(onehot_r, R0, R1, R2, R3, R4, R5, R6, R7, data_out);

endmodule

module Dec38(in, out);
input [2:0]in;
output reg [7:0]out;

always @(*) begin
    case(in)
    3'b000: out = 8'b00000001; //writenum is 0 -> load0 -> R0
    3'b001: out = 8'b00000010; //writenum is 1 -> load1 -> R1
    3'b010: out = 8'b00000100; //writenum is 2 -> load2 -> R2
    3'b011: out = 8'b00001000; //writenum is 3 -> load3 -> R3
    3'b100: out = 8'b00010000; //writenum is 4 -> load4 -> R4
    3'b101: out = 8'b00100000; //writenum is 5 -> load5 -> R5
    3'b110: out = 8'b01000000; //writenum is 6 -> load6 -> R6
    3'b111: out = 8'b10000000; //writenum is 7 -> load7 -> R7
    default: out = 8'bxxxxxxxx;
    endcase
end
endmodule

module anded(onehot_w, write, load);
input onehot_w;
input write;
output load;

assign load = onehot_w & write; // AND-gate

endmodule

/*
module anded(onehot_w, write, load);
input [7:0]onehot_w;
input write;
output reg [7:0]load;

always @(*) begin
    case(onehot_w)
    8'b00000001: begin
        if(write == 1'b1) begin
            load = 8'b00000001;
        end
        else begin
            load = 8'bxxxxxxxx;
        end
    end

    8'b00000010: begin
        if(write == 1'b1) begin
            load = 8'b00000010;
        end
        else begin
            load = 8'bxxxxxxxx;
        end
    end

    8'b00000100: begin
        if(write == 1'b1) begin
            load = 8'b00000100;
        end
        else begin
            load = 8'bxxxxxxxx;
        end
    end

    8'b00001000: begin
        if(write == 1'b1) begin
            load = 8'b00001000;
        end
        else begin
            load = 8'bxxxxxxxx;
        end
    end

    8'b00010000: begin
        if(write == 1'b1) begin
            load = 8'b00010000;
        end
        else begin
            load = 8'bxxxxxxxx;
        end
    end

    8'b00100000: begin
        if(write == 1'b1) begin
            load = 8'b00100000;
        end
        else begin
            load = 8'bxxxxxxxx;
        end
    end

    8'b01000000: begin
        if(write == 1'b1) begin
            load = 8'b01000000;
        end
        else begin
            load = 8'bxxxxxxxx;
        end
    end

    8'b10000000: begin
        if(write == 1'b1) begin
            load = 8'b10000000;
        end
        else begin
            load = 8'bxxxxxxxx;
        end
    end

    default: load = 8'bxxxxxxxx;

    endcase
end
endmodule
*/

/*
module reg0(data_in,load,clk,R0);
input [15:0]data_in;
input [7:0]load;
input clk;
output reg [15:0]R0;

reg [15:0]present_state;

always @(posedge clk) begin
    if(load == 8'b10000000) begin
        present_state = data_in;
    end
    else begin
        present_state = R0; //first run -> will be undefined... maybe?
    end
    R0 = present_state;
end
endmodule
*/

// module register(data_in,load,clk,R);
// input [15:0]data_in;
// input load;
// input clk;
// output [15:0]R;

module register(data_in,load,clk,R);
parameter n = 1;
input [n-1:0]data_in;
input load;
input clk;
output [n-1:0]R;

wire [n-1:0]next_state_R;

vDFFreg #(n) v0(clk, next_state_R, R); //is the flip-flop, the assign statement below is the MUX
assign next_state_R = load ? data_in : R; //(MUX) if load is 1, then data_in , if load is 0, then R0

endmodule

module vDFFreg(clk, in, out) ;  // changed name to vDFFreg from vDFF
  parameter n = 1; // width
  input clk ;
  input [n-1:0] in ;
  output [n-1:0] out ;
  reg [n-1:0] out ;

  always @(posedge clk)
    out = in ; //makes the next_state_R the out (present_state)
endmodule
/*
module reg1(data_in,load,clk,R1);
input [15:0]data_in;
input [7:0]load;
input clk;
output reg [15:0]R1;

reg [15:0]present_state;

always @(posedge clk) begin
    if(load == 8'b01000000) begin
        present_state = data_in;
    end
    else begin
        present_state = R1; //first run -> will be undefined... maybe?
    end
    R1 = present_state;
end
endmodule

module reg2(data_in,load,clk,R2);
input [15:0]data_in;
input [7:0]load;
input clk;
output reg [15:0]R2;

reg [15:0]present_state;

always @(posedge clk) begin
    if(load == 8'b00100000) begin
        present_state = data_in;
    end
    else begin
        present_state = R2; //first run -> will be undefined... maybe?
    end
    R2 = present_state;
end
endmodule

module reg3(data_in,load,clk,R3);
input [15:0]data_in;
input [7:0]load;
input clk;
output reg [15:0]R3;

reg [15:0]present_state;

always @(posedge clk) begin
    if(load == 8'b00010000) begin
        present_state = data_in;
    end
    else begin
        present_state = R3; //first run -> will be undefined... maybe?
    end
    R3 = present_state;
end
endmodule

module reg4(data_in,load,clk,R4);
input [15:0]data_in;
input [7:0]load;
input clk;
output reg [15:0]R4;

reg [15:0]present_state;

always @(posedge clk) begin
    if(load == 8'b00001000) begin
        present_state = data_in;
    end
    else begin
        present_state = R4; //first run -> will be undefined... maybe?
    end
    R4 = present_state;
end
endmodule

module reg5(data_in,load,clk,R5);
input [15:0]data_in;
input [7:0]load;
input clk;
output reg [15:0]R5;

reg [15:0]present_state;

always @(posedge clk) begin
    if(load == 8'b00000100) begin
        present_state = data_in;
    end
    else begin
        present_state = R5; //first run -> will be undefined... maybe?
    end
    R5 = present_state;
end
endmodule

module reg6(data_in,load,clk,R6);
input [15:0]data_in;
input [7:0]load;
input clk;
output reg [15:0]R6;

reg [15:0]present_state;

always @(posedge clk) begin
    if(load == 8'b00000010) begin
        present_state = data_in;
    end
    else begin
        present_state = R6; //first run -> will be undefined... maybe?
    end
    R6 = present_state;
end
endmodule

module reg7(data_in,load,clk,R7);
input [15:0]data_in;
input [7:0]load;
input clk;
output reg [15:0]R7;

reg [15:0]present_state;

always @(posedge clk) begin
    if(load == 8'b00000001) begin
        present_state = data_in;
    end
    else begin
        present_state = R7; //first run -> will be undefined... maybe?
    end
    R7 = present_state;
end
endmodule
*/
module Mux(onehot_r, R0, R1, R2, R3, R4, R5, R6, R7, data_out); //this is the giant MUX at the bottom of the internal structure of register file
input [7:0]onehot_r;
input [15:0]R0;
input [15:0]R1; 
input [15:0]R2; 
input [15:0]R3; 
input [15:0]R4; 
input [15:0]R5; 
input [15:0]R6; 
input [15:0]R7;
output reg [15:0]data_out;

always @(*) begin
    case(onehot_r) //readnum is decoded and becomes onehot_r which then chooses which R passes the MUX
    8'b00000001: data_out = R0; //if LSB is on, then R0 is chosen
    8'b00000010: data_out = R1;
    8'b00000100: data_out = R2;
    8'b00001000: data_out = R3;
    8'b00010000: data_out = R4;
    8'b00100000: data_out = R5;
    8'b01000000: data_out = R6;
    8'b10000000: data_out = R7; //if MSB is on, then R7 is chosen
    default: data_out = 8'bxxxxxxxx;
    endcase
end
endmodule
