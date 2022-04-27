// STATE ENCODING

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


// Testbench for lab7_top

module lab7_top_tb(); // testbench so no I/O

reg [3:0] sim_KEY;
reg [9:0] sim_SW;
// outputs are wires
wire [9:0] sim_LEDR;
wire [6:0] sim_HEX0, sim_HEX1, sim_HEX2, sim_HEX3, sim_HEX4, sim_HEX5;

reg err; // error check

// instantiation
lab7_top DUT(
  .KEY(sim_KEY),
  .SW(sim_SW),
  .LEDR(sim_LEDR),
  .HEX0(sim_HEX0),
  .HEX1(sim_HEX1),
  .HEX2(sim_HEX2),
  .HEX3(sim_HEX3),
  .HEX4(sim_HEX4),
  .HEX5(sim_HEX5)
);

// TASK

task output_checker;

input [4:0] expected_state;
input [1:0] expected_mem_cmd;
input [8:0] expected_mem_addr;
input [15:0] expected_write_data;

input [15:0] expected_register_value_R0;
input [15:0] expected_register_value_R1;
input [15:0] expected_register_value_R2;
input [15:0] expected_register_value_R3;
input [15:0] expected_register_value_R4;
input [15:0] expected_register_value_R5;
input [15:0] expected_register_value_R6;
input [15:0] expected_register_value_R7;


begin
  // checking state for FSM
  if (lab7_top_tb.DUT.CPU.SM.state !== expected_state) begin
        $display("ERROR, *** state is %b, expected %b", lab7_top_tb.DUT.CPU.SM.state, expected_state);
        
        err = 1'b1; // raise error
    
    end
  // checking mem_cmd
    if (lab7_top_tb.DUT.CPU.mem_cmd !== expected_mem_cmd) begin
        $display("ERROR, *** mem_cmd is %b, expected %b", lab7_top_tb.DUT.CPU.mem_cmd, expected_mem_cmd);
        
        err = 1'b1; // raise error
    
    end
  // checking mem_addr
 if (lab7_top_tb.DUT.CPU.mem_addr !== expected_mem_addr) begin
     $display("ERROR, *** mem_addr is %b, expected %b", lab7_top_tb.DUT.CPU.mem_addr, expected_mem_addr);
        
     err = 1'b1; // raise error

 end

  // checking write_data
if (lab7_top_tb.DUT.CPU.write_data !== expected_write_data) begin
    $display("ERROR, *** write_data is %b, expected %b", lab7_top_tb.DUT.CPU.write_data, expected_write_data);
 
    err = 1'b1; // raise error

end


// checking registers
if (lab7_top_tb.DUT.CPU.DP.REGFILE.R0 !== expected_register_value_R0) begin
    $display("ERROR, *** register 0 is %b, expected %b", lab7_top_tb.DUT.CPU.DP.REGFILE.R0, expected_register_value_R0);

  err = 1'b1; // raise error
 end

if (lab7_top_tb.DUT.CPU.DP.REGFILE.R1 !== expected_register_value_R1) begin
    $display("ERROR, *** register 1 is %b, expected %b", lab7_top_tb.DUT.CPU.DP.REGFILE.R1, expected_register_value_R1);

      err = 1'b1; // raise error
    end

if (lab7_top_tb.DUT.CPU.DP.REGFILE.R2 !== expected_register_value_R2) begin
    $display("ERROR, *** register 2 is %b, expected %b", lab7_top_tb.DUT.CPU.DP.REGFILE.R2, expected_register_value_R2);

    err = 1'b1; // raise error
  end

      if (lab7_top_tb.DUT.CPU.DP.REGFILE.R3 !== expected_register_value_R3) begin
         $display("ERROR, *** register 3 is %b, expected %b", lab7_top_tb.DUT.CPU.DP.REGFILE.R3, expected_register_value_R3);

         err = 1'b1; // raise error
     end
    
      if (lab7_top_tb.DUT.CPU.DP.REGFILE.R4 !== expected_register_value_R4) begin
         $display("ERROR, *** register 4 is %b, expected %b", lab7_top_tb.DUT.CPU.DP.REGFILE.R4, expected_register_value_R4);

         err = 1'b1; // raise error
     end

      if (lab7_top_tb.DUT.CPU.DP.REGFILE.R5 !== expected_register_value_R5) begin
         $display("ERROR, *** register 5 is %b, expected %b", lab7_top_tb.DUT.CPU.DP.REGFILE.R5, expected_register_value_R5);

         err = 1'b1; // raise error
     end

     if (lab7_top_tb.DUT.CPU.DP.REGFILE.R6 !== expected_register_value_R6) begin
        $display("ERROR, *** register 6 is %b, expected %b", lab7_top_tb.DUT.CPU.DP.REGFILE.R6, expected_register_value_R6);

        err = 1'b1; // raise error
    end

     if (lab7_top_tb.DUT.CPU.DP.REGFILE.R7 !== expected_register_value_R7) begin
        $display("ERROR, *** register 7 is %b, expected %b", lab7_top_tb.DUT.CPU.DP.REGFILE.R7, expected_register_value_R7);

        err = 1'b1; // raise error
    end

// checking if ARM command in data_DE1.txt matches in memory
  if (DUT.MEM.mem[0] !== 16'b1101000000001000) begin // @00
    $display("FAILED: mem[0] wrong; please set data_DE1.txt using Figure 8");
    
    err = 1'b1;

    end
    
  if (DUT.MEM.mem[1] !== 16'b0110000000000000) begin // @01
    $display("FAILED: mem[1] wrong; please set data_DE1.txt using Figure 8"); 
    
    err = 1;

    end

  if (DUT.MEM.mem[2] !== 16'b0110000001000000) begin // @02
  $display("FAILED: mem[2] wrong; please set data_DE1.txt using Figure 8"); 
  
  err = 1;

  end

  if (DUT.MEM.mem[3] !== 16'b1100000001101010) begin // @03
  $display("FAILED: mem[3] wrong; please set data_DE1.txt using Figure 8"); 
  
  err = 1;

  end

  if (DUT.MEM.mem[4] !== 16'b1101000100001001) begin // @04
  $display("FAILED: mem[4] wrong; please set data_DE1.txt using Figure 8"); 
  
  err = 1;

  end

  if (DUT.MEM.mem[5] !== 16'b0110000100100000) begin // @05
  $display("FAILED: mem[5] wrong; please set data_DE1.txt using Figure 8"); 
  
  err = 1;

  end

  if (DUT.MEM.mem[6] !== 16'b1000000101100000) begin // @06
  $display("FAILED: mem[6] wrong; please set data_DE1.txt using Figure 8"); 
  
  err = 1;

  end

  if (DUT.MEM.mem[7] !== 16'b1110000000000000) begin // @07
  $display("FAILED: mem[7] wrong; please set data_DE1.txt using Figure 8"); 
  
  err = 1;

  end

end
endtask

// START DEBUG

 initial forever begin // starting clk
    sim_KEY[0] = 1; #5;
    sim_KEY[0] = 0; #5;
  end

initial begin
  err = 0;
  sim_KEY[1] = 1'b0; // reset asserted

#10; // wait until next falling edge of clock
sim_KEY[1] = 1'b1; // reset de-asserted, PC still undefined if as in Figure 4

// ___________________________________________________ NEW ARM COMMAND __________________________________________________

// MOV R0, SW_BASE needs 7 clk cycles
$display(" Checking MOV R0, SW_BASE. Part 1");
output_checker(`RST, `MNONE, 9'bxxxxxxxxx, 16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R0, SW_BASE. Part 2");
#10;
output_checker(`IF1, `MREAD, 9'b000000000, 16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R0, SW_BASE. Part 3");
#10;
output_checker(`IF2, `MREAD, 9'b000000000, 16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R0, SW_BASE. Part 4");
#10;
output_checker(`UpdatePC, `MNONE, 9'bxxxxxxxxx, 16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R0, SW_BASE. Part 5");
#10;
output_checker(`Decode, `MNONE, 9'bxxxxxxxxx, 16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R0, SW_BASE. Part 6");
#10;
output_checker(`Writelmm, `MNONE, 9'bxxxxxxxxx, 16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R0, SW_BASE. Part 7");
#10;
output_checker(`IF1, `MREAD, 9'b000000001, 16'bxxxxxxxxxxxxxxxx,
16'b0000000000001000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

// ___________________________________________________ NEW ARM COMMAND __________________________________________________
// LDR R0, [R0] needs 9 clk cycles
$display(" Checking LDR R0, [R0]. Part 1");
#10;
output_checker(`IF2, `MREAD, 9'b000000001, 16'bxxxxxxxxxxxxxxxx,
16'b0000000000001000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R0, [R0]. Part 2");
#10;
output_checker(`UpdatePC, `MNONE, 9'bxxxxxxxxx, 16'bxxxxxxxxxxxxxxxx,
16'b0000000000001000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R0, [R0]. Part 3");
#10;
output_checker(`Decode, `MNONE, 9'bxxxxxxxxx, 16'bxxxxxxxxxxxxxxxx,
16'b0000000000001000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R0, [R0]. Part 4");
#10;
output_checker(`GetA, `MNONE, 9'bxxxxxxxxx, 16'bxxxxxxxxxxxxxxxx,
16'b0000000000001000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R0, [R0]. Part 5");
#10;
output_checker(`LDR, `MNONE, 9'bxxxxxxxxx, 16'bxxxxxxxxxxxxxxxx,
16'b0000000000001000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R0, [R0]. Part 6");
#10;
output_checker(`LDR_MEM_DA, `MNONE, 9'bxxxxxxxxx, 16'b0000000000001000, 
16'b0000000000001000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R0, [R0]. Part 7");
#10;
output_checker(`LDR_MEM_MDATA, `MREAD, 9'b000001000, 16'b0000000000001000,
16'b0000000000001000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R0, [R0]. Part 8");
#10;
output_checker(`LDR_MEM_Rd, `MREAD, 9'b000001000, 16'b0000000000001000,
16'b0000000000001000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R0, [R0]. Part 9");
#10;
output_checker(`IF1, `MREAD, 9'b000000010, 16'b0000000000001000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

// ___________________________________________________ NEW ARM COMMAND __________________________________________________
// LDR R2, [R0] needs 9 clk cycles
$display(" Checking LDR R2, [R0]. Part 1");
#10;
output_checker(`IF2, `MREAD, 9'b000000010, 16'b0000000000001000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R2, [R0]. Part 2");
#10;
output_checker(`UpdatePC, `MNONE, 9'b000001000, 16'b0000000000001000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R2, [R0]. Part 3");
#10
output_checker(`Decode, `MNONE, 9'b000001000, 16'b0000000000001000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R2, [R0]. Part 4");
#10;
output_checker(`GetA, `MNONE, 9'b000001000, 16'b0000000000001000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R2, [R0]. Part 5");
#10;
output_checker(`LDR, `MNONE, 9'b000001000, 16'b0000000000001000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R2, [R0]. Part 6");
#10;
output_checker(`LDR_MEM_DA, `MNONE, 9'b000001000, 16'b0000000101000000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R2, [R0]. Part 7");
#10;
output_checker(`LDR_MEM_MDATA, `MREAD, 9'b101000000, 16'b0000000101000000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R2, [R0]. Part 8");
#10;
output_checker(`LDR_MEM_Rd, `MREAD, 9'b101000000, 16'b0000000101000000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R2, [R0]. Part 9");
#10;
output_checker(`IF1, `MREAD, 9'b000000011, 16'b0000000101000000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

// ___________________________________________________ NEW ARM COMMAND __________________________________________________
// MOV R3, R2, LSL #1 needs 8 clk cycles 
$display(" Checking MOV R3, R2, LSL #1. Part 1");
#10;
output_checker(`IF2, `MREAD, 9'b000000011, 16'b0000000101000000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R3, R2, LSL #1. Part 2");
#10;
output_checker(`UpdatePC, `MNONE, 9'b101000000, 16'b0000000101000000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R3, R2, LSL #1. Part 3");
#10;
output_checker(`Decode, `MNONE, 9'b101000000, 16'b0000000101000000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R3, R2, LSL #1. Part 4");
#10;
output_checker(`GetA, `MNONE, 9'b101000000, 16'b0000000101000000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R3, R2, LSL #1. Part 5");
#10;
output_checker(`GetB, `MNONE, 9'b101000000, 16'b0000000101000000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R3, R2, LSL #1. Part 6");
#10;
output_checker(`MOV_Rd_Rm, `MNONE, 9'b101000000, 16'b0000000101000000,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R3, R2, LSL #1. Part 7");
#10;
output_checker(`WriteReg, `MNONE, 9'b101000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R3, R2, LSL #1. Part 8");
#10;
output_checker(`IF1, `MREAD, 9'b000000100, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

// ___________________________________________________ NEW ARM COMMAND __________________________________________________
// MOV R1, LEDR_BASE needs 5 clk cycles (because not in RST state and IF1 state)
$display(" Checking MOV R1, LEDR_BASE. Part 1");
#10;
output_checker(`IF2, `MREAD, 9'b000000100, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R1, LEDR_BASE. Part 2");
#10;
output_checker(`UpdatePC, `MNONE, 9'b101000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R1, LEDR_BASE. Part 3");
#10;
output_checker(`Decode, `MNONE, 9'b101000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R1, LEDR_BASE. Part 4");
#10;
output_checker(`Writelmm, `MNONE, 9'b101000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'bxxxxxxxxxxxxxxxx,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking MOV R1, LEDR_BASE. Part 5");
#10;
output_checker(`IF1, `MREAD, 9'b000000101, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000000001001,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

// ___________________________________________________ NEW ARM COMMAND __________________________________________________
// LDR R1, [R1] needs 9 clk cycles
$display(" Checking LDR R1, [R1]. Part 1");
#10;
output_checker(`IF2, `MREAD, 9'b000000101, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000000001001,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R1, [R1]. Part 2");
#10;
output_checker(`UpdatePC, `MNONE, 9'b101000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000000001001,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R1, [R1]. Part 3");
#10;
output_checker(`Decode, `MNONE, 9'b101000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000000001001,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R1, [R1]. Part 4");
#10;
output_checker(`GetA, `MNONE, 9'b101000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000000001001,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R1, [R1]. Part 5");
#10;
output_checker(`LDR, `MNONE, 9'b101000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000000001001,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R1, [R1]. Part 6");
#10;
output_checker(`LDR_MEM_DA, `MNONE, 9'b101000000, 16'b0000000000001001,
16'b0000000101000000,16'b0000000000001001,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R1, [R1]. Part 7");
#10;
output_checker(`LDR_MEM_MDATA, `MREAD, 9'b000001001, 16'b0000000000001001,
16'b0000000101000000,16'b0000000000001001,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R1, [R1]. Part 8");
#10;
output_checker(`LDR_MEM_Rd, `MREAD, 9'b000001001, 16'b0000000000001001,
16'b0000000101000000,16'b0000000000001001,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking LDR R1, [R1]. Part 9");
#10;
output_checker(`IF1, `MREAD, 9'b000000110, 16'b0000000000001001,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);


// ___________________________________________________ NEW ARM COMMAND __________________________________________________
// STR R3, [R1] needs 10 clk cycles
$display(" Checking STR R3, [R1]. Part 1");
#10;
output_checker(`IF2, `MREAD, 9'b000000110, 16'b0000000000001001,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking STR R3, [R1]. Part 2");
#10;
output_checker(`UpdatePC, `MNONE, 9'b000001001, 16'b0000000000001001,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking STR R3, [R1]. Part 3");
#10;
output_checker(`Decode, `MNONE, 9'b000001001, 16'b0000000000001001,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking STR R3, [R1]. Part 4");
#10;
output_checker(`GetA, `MNONE, 9'b000001001, 16'b0000000000001001,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking STR R3, [R1]. Part 5");
#10;
output_checker(`GetRd, `MNONE, 9'b000001001, 16'b0000000000001001,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking STR R3, [R1]. Part 6");
#10;
output_checker(`STR, `MNONE, 9'b000001001, 16'b0000000000001001,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking STR R3, [R1]. Part 7");
#10;
output_checker(`STR_MEM_DA, `MNONE, 9'b000001001, 16'b0000000100000000,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking STR R3, [R1]. Part 8");
#10;
output_checker(`STR_MEM_DIN, `MNONE, 9'b100000000, 16'b0000000100000000,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking STR R3, [R1]. Part 9");
#10;
output_checker(`STR_MEM_RAM, `MWRITE, 9'b100000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking STR R3, [R1]. Part 10");
#10;
output_checker(`IF1, `MREAD, 9'b000000111, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

// ___________________________________________________ NEW ARM COMMAND __________________________________________________
// HALT needs 5 clk cycles (last cycle is going back to RST)
$display(" Checking HALT. Part 1");
#10;
output_checker(`IF2, `MREAD, 9'b000000111, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking HALT. Part 2");
#10;
output_checker(`UpdatePC, `MNONE, 9'b100000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking HALT. Part 3");
#10;
output_checker(`Decode, `MNONE, 9'b100000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

$display(" Checking HALT. Part 4");
#10;
output_checker(`HALT, `MNONE, 9'b100000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

sim_KEY[1] = 1'b0; // reset asserted

#10;
$display(" Checking HALT. Part 5");
output_checker(`RST, `MNONE, 9'b100000000, 16'bxxxxxxxxxxxxxxxx,
16'b0000000101000000,16'b0000000100000000,16'b00000000xxxxxxxx,16'bxxxxxxxxxxxxxxxx,
16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx);

if (~err) $display("PASSED"); 
    else $display("FAILED");
    $stop;

end

endmodule

