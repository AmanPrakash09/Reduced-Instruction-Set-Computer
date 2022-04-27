// lab7_top module

// defining HEX numbers and letters
`define OFF 7'b1111111      //nothing is displayed
`define ZERO 7'b1000000         // 0
`define ONE 7'b1111001          // 1

`define E 7'b0000110
`define R 7'b0101111 
`define O 7'b1000000

// state encoding:
`define RST 5'b00000  // 1st
`define Decode 5'b00001 // 5th
`define GetA 5'b00010
`define GetB 5'b00011
`define GetRd 5'b10111 // load Rd into Register B

`define MOV_Rd_Rm 5'b00100
`define ADD 5'b00101
`define CMP 5'b00110
`define AND 5'b00111 
`define MVN 5'b01000
`define LDR 5'b01001 // LDR branch for loading R[rn] + sx(im5) into register C
`define STR 5'b01010
`define LDR_MEM_DA 5'b01011 // LDR branch for loading R[rn] + sx(im5) into Data Address register
`define LDR_MEM_MDATA 5'b01100 // LDR branch for setting mem_cmd to MREAD and addr_sel to 0 so that mdata gets value from memory
`define LDR_MEM_Rd 5'b01101 // LDR branch for writing M [ R[rn] + sx(im5) ] into Rd

`define STR_MEM_DA 5'b01110 // STR branch for loading R[rn] + sx(im5) into Data Address register
`define STR_MEM_DIN 5'b01111 // STR branch for keeping R[rn] + sx(im5) into Data Address register and changing datapath_out
`define STR_MEM_RAM 5'b10000 // STR branch for loading Rd into address R[rn] + sx(im5) into RAM

`define WriteReg 5'b10001
`define Writelmm 5'b10010

// NEW ADDITIONAL STATES LAB7

`define IF1 5'b10011 // 2nd
`define IF2 5'b10100 // 3rd
`define UpdatePC 5'b10101 // 4th
`define HALT 5'b10110

`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10

module lab7_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5);
input [3:0] KEY;
input [9:0] SW;
output [9:0] LEDR;
output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

// // wires
wire clk = ~KEY[0];
wire reset = ~KEY[1];

// // assigning wires
// assign clk = ~KEY[0];
// assign reset = ~KEY[1];

//NOTE: I THINK WE NEED TO MAKE A MODULE SO THAT THE DECLRATIONS CAN BE WIRES

// wire [15:0] read_data; // 16 bits wide
// wire [15:0] write_dSata; // 16 bits wide
// reg [8:0] mem_addr; // 9 bits wide
// reg [1:0] mem_cmd; // 2 bits wide
// wire N, V, Z;

// reg mem_cmd_read;
// reg mem_cmd_write;
// reg msel;

// reg enable; // for the tri-state
// reg write; // for write input to RAM

// wire [15:0] dout; // output from RAM into tri-state


wire [15:0] read_data; // 16 bits wide
wire [15:0] write_data; // 16 bits wide
wire [8:0] mem_addr; // 9 bits wide
wire [1:0] mem_cmd; // 2 bits wide
wire N, V, Z;

wire mem_cmd_read; // wire coming out of AND gate
wire mem_cmd_write; // wire coming out of AND gate
wire msel; // wire coming out of AND gate

wire enable; // for the tri-state
wire write; // for write input to RAM

wire [15:0] dout; // output from RAM into tri-state


// instantiations + combinational: 

// cpu module
cpu CPU(.clk(clk),.reset(reset),.read_data(read_data),.write_data(write_data),.mem_addr(mem_addr),.mem_cmd(mem_cmd),.N(N),.V(V),.Z(Z));

// instantiations for RAM:
equality_MWRITE_MREAD MWRITE(mem_cmd, `MWRITE, mem_cmd_write); // checking if equal
equality_MWRITE_MREAD MREAD(mem_cmd, `MREAD, mem_cmd_read); // checking if equal
equality_msel mselect(mem_addr[8],msel); // checking if equal

// always @(*) begin // equal to sign for MREAD
//     if ( mem_cmd == `MREAD) begin
//         mem_cmd_read = 1'b1;
//     end else begin
//         mem_cmd_read = 1'b0;
//     end
// end

// always @(*) begin // equal to sign for MWRITE
//     if ( mem_cmd == `MWRITE) begin
//         mem_cmd_write = 1'b1;
//     end else begin
//         mem_cmd_write = 1'b0;
//     end
// end

// always @(*) begin // equal to sign for msel
//     if (mem_addr[8:8] == 1'b0) begin
//         msel = 1'b1;
//     end else begin
//         msel = 1'b0;
//     end
// end

// anded READ(mem_cmd_read, msel, enable);

// anded WRITE(mem_cmd_write, msel, write);

// always @(*) begin
//     enable = mem_cmd_read & msel;
//     write = mem_cmd_write & msel;
// end

// RAM combinational:
assign write = mem_cmd_write & msel; // wire coming out of AND gate

assign read_data = (mem_cmd_read & msel) ? dout : {16{1'bz}}; // tristate

// RAM instantiation
RAM MEM(.clk(~KEY[0]),.read_address(mem_addr[7:0]),.write_address(mem_addr[7:0]),.write(write),.din(write_data),.dout(dout));

// STAGE 3 Combinational/Instantiations:

wire swcomb_out; // wire coming out of combinational circuit

wire ledscomb_out; // wire coming out of combinational circuit

// combinational instantiation
SWITCHES SWCOMB(.mem_cmd(mem_cmd),.mem_addr(mem_addr),.out(swcomb_out));

// assign read_data = swcomb_out ? {8'h00,SW[7:0]} : {16{1'bz}};

assign read_data[15:8] = swcomb_out ? 8'h00 : {8{1'bz}}; // tristate
assign read_data[7:0] = swcomb_out ? SW[7:0] : {8{1'bz}}; // tristate

// combinational instantiation
LEDS LEDSCOMB(.mem_cmd(mem_cmd),.mem_addr(mem_addr),.out(ledscomb_out));

// register instantiation
registerCPU #(8) LEDREG(write_data[7:0],ledscomb_out,clk,LEDR[7:0]);

// DISPLAY HD(HEX5,HEX4,HEX3,HEX2,HEX1,HEX0);

endmodule


// MODULE DECLARIATIONS:    

// To ensure Quartus uses the embedded MLAB memory blocks inside the Cyclone
// V on your DE1-SoC we follow the coding style from in Altera's Quartus II
// Handbook (QII5V1 2015.05.04) in Chapter 12, “Recommended HDL Coding Style”
//
// 1. "Example 12-11: Verilog Single Clock Simple Dual-Port Synchronous RAM 
//     with Old Data Read-During-Write Behavior" 
// 2. "Example 12-29: Verilog HDL RAM Initialized with the readmemb Command"

module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 16; 
  parameter addr_width = 8;
  parameter filename = "data.txt"; // will need to change to data_DE1 for our own DE1 demonstration

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @ (posedge clk) begin
    if (write)
      mem[write_address] <= din;
    dout <= mem[read_address]; // dout doesn't get din in this clock cycle 
                               // (this is due to Verilog non-blocking assignment "<=")
  end 
endmodule

// STAGE 3 modules:

// module for switches:

module SWITCHES(mem_cmd, mem_addr, out);

input [1:0] mem_cmd;
input [8:0] mem_addr;

output reg out;

always @(*) begin // out = 1'b1 if mem_cmd == `MREAD && mem_addr == 9'h140
    if (mem_cmd == `MREAD && mem_addr == 9'h140) begin
        out = 1'b1;
    end
    else begin
        out = 1'b0;
    end
end

endmodule


module LEDS(mem_cmd, mem_addr, out);

input [1:0] mem_cmd;
input [8:0] mem_addr;

output reg out;

always @(*) begin // out = 1'b1 if mem_cmd == `MWRITE && mem_addr == 9'h100
    if (mem_cmd == `MWRITE && mem_addr == 9'h100) begin
        out = 1'b1;
    end
    else begin
        out = 1'b0;
    end
end

endmodule




// module for equal comparisons

module equality_MWRITE_MREAD(mem_cmd, comparison, out);

input [1:0] mem_cmd;
input [1:0] comparison; // will be either MWRITE or MREAD 
output reg out;
always @(*) begin
    if (mem_cmd == comparison) begin
        out = 1'b1;
    end
    else begin
        out = 1'b0;
    end 
end


endmodule


module equality_msel(mem_addr,out);

input mem_addr; // left most bit of mem_addr
output reg out;

always @(*) begin
    if (mem_addr == 1'b0) begin
        out = 1'b1;
    end
    else begin
        out = 1'b0;
    end    
end

endmodule


// module DISPLAY(HEX5,HEX4,HEX3,HEX2,HEX1,HEX0);

// output reg [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;


// always @(*) begin
//     case(lab7_top.CPU.SM.state)
//     `RST:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ZERO,`ZERO,`ZERO,`ZERO};
//     `Decode:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ZERO,`ZERO,`ZERO,`ONE};
//     `GetA:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ZERO,`ZERO,`ONE,`ZERO};
//     `GetB:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ZERO,`ZERO,`ONE,`ONE};
//     `GetRd:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ONE,`ZERO,`ONE,`ONE,`ONE};
//     `MOV_Rd_Rm:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ZERO,`ONE,`ZERO,`ZERO};
//     `ADD:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ZERO,`ONE,`ZERO,`ONE};
//     `CMP:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ZERO,`ONE,`ONE,`ZERO};
//     `AND:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ZERO,`ONE,`ONE,`ONE};
//     `MVN:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ONE,`ZERO,`ZERO,`ZERO};
//     `LDR:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ONE,`ZERO,`ZERO,`ONE};
//     `STR:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ONE,`ZERO,`ONE,`ZERO};
//     `LDR_MEM_DA:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ONE,`ZERO,`ONE,`ONE};
//     `LDR_MEM_MDATA:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ONE,`ONE,`ZERO,`ZERO};
//     `LDR_MEM_Rd:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ONE,`ONE,`ZERO,`ONE};
//     `STR_MEM_DA:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ONE,`ONE,`ONE,`ZERO};
//     `STR_MEM_DIN:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ZERO,`ONE,`ONE,`ONE,`ONE};
//     `STR_MEM_RAM:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ONE,`ZERO,`ZERO,`ZERO,`ZERO};
//     `WriteReg:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ONE,`ZERO,`ZERO,`ZERO,`ONE};
//     `Writelmm:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ONE,`ZERO,`ZERO,`ONE,`ZERO};
//     `IF1:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ONE,`ZERO,`ZERO,`ONE,`ONE};
//     `IF2:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ONE,`ZERO,`ONE,`ZERO,`ZERO};
//     `UpdatePC:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ONE,`ZERO,`ONE,`ZERO,`ONE};
//     `HALT:{HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`ONE,`ZERO,`ONE,`ONE,`ZERO};
//     default: {HEX5,HEX4,HEX3,HEX2,HEX1,HEX0} = {`OFF,`E,`R,`R,`O,`R};
//     endcase    

// end

// endmodule

