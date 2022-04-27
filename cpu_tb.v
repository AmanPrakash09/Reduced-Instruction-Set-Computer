
// testbench for LAB6 cpu MOV function

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



module cpu_tb(); // testbench so no I/O

// inputs
reg sim_clk;
reg sim_reset;
reg [15:0] sim_read_data; 

// outputs
wire [15:0] sim_write_data; 
wire [8:0] sim_mem_addr; 
wire [1:0] sim_mem_cmd; 
wire N, V, Z;

reg err; // error check

// INSTANTIATION

cpu DUT(
    .clk(sim_clk),
    .reset(sim_reset),
    .read_data(sim_read_data),
    .write_data(sim_write_data),
    .mem_addr(sim_mem_addr),
    .mem_cmd(sim_mem_cmd),
    .N(sim_N),
    .V(sim_V),
    .Z(sim_Z)
);

//TASK

task output_checker;


    input [4:0] expected_state;

    input [1:0] expected_mem_cmd;

    // input [8:0] expected_mem_addr;
    // input [15:0] expected_write_data;

    // input [15:0] expected_register_value_R0;
    // input [15:0] expected_register_value_R1;
    // input [15:0] expected_register_value_R2;
    // input [15:0] expected_register_value_R3;
    // input [15:0] expected_register_value_R4;
    // input [15:0] expected_register_value_R5;
    // input [15:0] expected_register_value_R6;
    // input [15:0] expected_register_value_R7;

   

begin

    if (cpu_tb.DUT.SM.state !== expected_state) begin
        $display("ERROR, *** state is %b, expected %b", cpu_tb.DUT.SM.state, expected_state);
        
        err = 1'b1; // raise error
    
    end

    if (sim_mem_cmd !== expected_mem_cmd) begin
        $display("ERROR, *** mem_cmd is %b, expected %b", sim_mem_cmd, expected_mem_cmd);
        
        err = 1'b1; // raise error
    
    end

    // if (sim_mem_addr !== expected_mem_addr) begin
    //     $display("ERROR, *** mem_addr is %b, expected %b", sim_mem_addr, expected_mem_addr);
        
    //     err = 1'b1; // raise error
    
    // end

    // if (sim_write_data !== expected_write_data) begin
    //     $display("ERROR, *** write_data is %b, expected %b", sim_write_data, expected_write_data);
        
    //     err = 1'b1; // raise error
    
    // end



    // if (cpu_tb.DUT.DP.REGFILE.R0 !== expected_register_value_R0) begin
    //     $display("ERROR, *** register 0 is %b, expected %b", cpu_tb.DUT.DP.REGFILE.R0, expected_register_value_R0);

    //     err = 1'b1; // raise error
    // end

    //  if (cpu_tb.DUT.DP.REGFILE.R1 !== expected_register_value_R1) begin
    //     $display("ERROR, *** register 1 is %b, expected %b", cpu_tb.DUT.DP.REGFILE.R1, expected_register_value_R1);

    //     err = 1'b1; // raise error
    // end

    //  if (cpu_tb.DUT.DP.REGFILE.R2 !== expected_register_value_R2) begin
    //     $display("ERROR, *** register 2 is %b, expected %b", cpu_tb.DUT.DP.REGFILE.R2, expected_register_value_R2);

    //     err = 1'b1; // raise error
    // end

    //  if (cpu_tb.DUT.DP.REGFILE.R3 !== expected_register_value_R3) begin
    //     $display("ERROR, *** register 3 is %b, expected %b", cpu_tb.DUT.DP.REGFILE.R3, expected_register_value_R3);

    //     err = 1'b1; // raise error
    // end
    
    //  if (cpu_tb.DUT.DP.REGFILE.R4 !== expected_register_value_R4) begin
    //     $display("ERROR, *** register 4 is %b, expected %b", cpu_tb.DUT.DP.REGFILE.R4, expected_register_value_R4);

    //     err = 1'b1; // raise error
    // end

    //  if (cpu_tb.DUT.DP.REGFILE.R5 !== expected_register_value_R5) begin
    //     $display("ERROR, *** register 5 is %b, expected %b", cpu_tb.DUT.DP.REGFILE.R5, expected_register_value_R5);

    //     err = 1'b1; // raise error
    // end

    //  if (cpu_tb.DUT.DP.REGFILE.R6 !== expected_register_value_R6) begin
    //     $display("ERROR, *** register 6 is %b, expected %b", cpu_tb.DUT.DP.REGFILE.R6, expected_register_value_R6);

    //     err = 1'b1; // raise error
    // end

    //  if (cpu_tb.DUT.DP.REGFILE.R7 !== expected_register_value_R7) begin
    //     $display("ERROR, *** register 7 is %b, expected %b", cpu_tb.DUT.DP.REGFILE.R7, expected_register_value_R7);

    //     err = 1'b1; // raise error
    // end

        
end
endtask


// START DEBUG

initial begin       // clock timing
    sim_clk = 1'b0; #5;
    forever begin
        sim_clk = 1'b1; #5;
        sim_clk = 1'b0; #5;

    end

end


initial begin


err = 1'b0; // set error to 0

sim_reset = 1'b1; // push reset

#10; // wait for clk

sim_reset = 1'b0; // deassert reset


$display("--- Checking MOV Rn#<im8>, MOV R0, #7 instruction --- ");

$display("Checking input instruction encoding");
sim_read_data = 16'b1101000000000111;                // MOV R0, #7 instruction encoded

$display("Checking RST");

output_checker(`RST, `MNONE); 

$display("Checking RST -> IF1");
#10

output_checker(`IF1, `MREAD); 

$display("Checking IF1 -> IF2");
#10

output_checker(`IF2, `MREAD); 

$display("Checking IF2 -> UpdatePC");
#10

output_checker(`UpdatePC, `MNONE); 

$display("Checking UpdatePC -> Decode");
#10

output_checker(`Decode, `MNONE); 

$display("Checking Decode -> Writelmm");
#10

output_checker(`Writelmm, `MNONE); 

$display("Checking Writelmm -> IF1");
#10

output_checker(`IF1, `MREAD); 

// NEW TEST // 

$display("--- Checking LDR Rd,[Rn{,#<im5>}], LDR R1, [R0, #5] instruction --- ");

$display("Checking input instruction encoding");
sim_read_data = 16'b0110000000100101;                // MOV R0, #7 instruction encoded


$display("Checking IF1 -> IF2");
#10

output_checker(`IF2, `MREAD); 

$display("Checking IF2 -> UpdatePC");
#10

output_checker(`UpdatePC, `MNONE); 

$display("Checking UpdatePC -> Decode");
#10

output_checker(`Decode, `MNONE);

$display("Checking Decode -> GetA");
#10

output_checker(`GetA, `MNONE);

$display("Checking GetA -> LDR");
#10

output_checker(`LDR, `MNONE);

$display("Checking LDR -> LDR_MEM_DA");
#10

output_checker(`LDR_MEM_DA, `MNONE);

$display("Checking LDR_MEM_DA -> LDR_MEM_MDATA");
#10

output_checker(`LDR_MEM_MDATA, `MREAD);

$display("Checking LDR_MEM_MDATA -> LDR_MEM_Rd");
#10

output_checker(`LDR_MEM_Rd, `MREAD);

$display("Checking LDR_MEM_Rd -> IF1");
#10

output_checker(`IF1, `MREAD);



// NEW TEST // 

$display("--- Checking STR Rd,[Rn{,#<im5>}], STR R1, [R0, #5] instruction --- ");

$display("Checking input instruction encoding");
sim_read_data = 16'b1000000000100101;                


$display("Checking IF1 -> IF2");
#10

output_checker(`IF2, `MREAD); 

$display("Checking IF2 -> UpdatePC");
#10

output_checker(`UpdatePC, `MNONE); 

$display("Checking UpdatePC -> Decode");
#10

output_checker(`Decode, `MNONE);

$display("Checking Decode -> GetA");
#10

output_checker(`GetA, `MNONE);

$display("Checking GetA -> GetRd");
#10

output_checker(`GetRd, `MNONE);

$display("Checking GetRd -> STR");
#10

output_checker(`STR, `MNONE);

$display("Checking STR -> STR_MEM_DA");
#10

output_checker(`STR_MEM_DA, `MNONE);

$display("Checking STR_MEM_DA -> STR_MEM_DIN");
#10

output_checker(`STR_MEM_DIN, `MNONE);

$display("Checking STR_MEM_DIN -> STR_MEM_RAM");
#10

output_checker(`STR_MEM_RAM, `MWRITE);

$display("Checking STR_MEM_RAM -> IF1");
#10

output_checker(`IF1, `MREAD);

// $display("Checking Decode -> Writelmm");
// #10; 
// output_checker(1'bx,1'bx,1'bx,`Writelmm,
//                 16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
//                 1'b0); // State should be in Writelmm, R0 undefined, w = 0 as not in wait state
//                                                      // R0 is undefined because of the delay of change in 'state', 
//                                                      // causing 'readnum' change delay which is no longer on rising edge of clock
//                                                      // Therefore R0 will be updated on next rising edge of clock

// $display("Checking Writelmm -> Wait ... Checking R0 = 7");
// #10; 
// output_checker(1'bx,1'bx,1'bx,`Wait,
//                 16'b0000000000000111,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx,
//                 1'b1); // State should be in Writelmm, R0 = 7, w = 1 as in wait state


if (~err) $display("PASSED"); 
    else $display("FAILED");
    $stop;




end

endmodule



