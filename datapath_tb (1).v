
// testbench for LAB5 datapath




module datapath_tb(); // no I/O because testbench

reg [15:0] sim_mdata;
reg [15:0] sim_sximm8;
//reg [7:0] sim_PC;
wire [15:0] sim_datapath_out; // output is a wire
reg [2:0] sim_writenum;
reg [2:0] sim_readnum;
reg sim_write;
reg [1:0] sim_vsel;
reg sim_asel;
reg sim_bsel;
reg [15:0] sim_sximm5;
reg sim_loada;
reg sim_loadb;
reg sim_loadc;
reg sim_loads;
reg [1:0] sim_shift;
reg [1:0] sim_ALUop;
wire sim_N; 
wire sim_V; 
wire sim_Z;             // output is a wire
reg sim_clk;

reg err; // error check

// INSTANTIATION

datapath DUT(
    .mdata(sim_mdata),
    .sximm8(sim_sximm8),
    //.PC(sim_PC),
    .datapath_out(sim_datapath_out),
    .readnum(sim_readnum),
    .writenum(sim_writenum),
    .write(sim_write),
    .vsel(sim_vsel),
    .asel(sim_asel),
    .bsel(sim_bsel),
    .sximm5(sim_sximm5),
    .loada(sim_loada),
    .loadb(sim_loadb),
    .loadc(sim_loadc),
    .loads(sim_loads),
    .shift(sim_shift),
    .ALUop(sim_ALUop),
    .N(sim_N),
    .V(sim_V),
    .Z(sim_Z),
    .clk(sim_clk) 
);

//TASK

task output_checker;
    input [15:0] expected_datapath_out;
    input expected_N;
    input expected_V;
    input expected_Z;
    input [15:0] expected_data_out;

    //input [15:0] expected_loadc_out;

    // input [15:0] expected_loada_out; // taken out because we cant use hierarchical names
    // input [15:0] expected_loadb_out;

begin
    if (sim_datapath_out !== expected_datapath_out) begin
        $display("ERROR, *** datapath_out is %b, expected %b", sim_datapath_out, expected_datapath_out); //if expected datapath_out does not match actual datapath_out, then err = 1
        
        err = 1'b1; // raise error
    
    end
    
    if (sim_N !== expected_N) begin
        $display("ERROR, *** N_z1 is %b, expected %b", sim_N, expected_N); //if expected N_z1 does not match actual N_z1, then err = 1
        
        err = 1'b1; // raise error
    
    end

    if (sim_V !== expected_V) begin
        $display("ERROR, *** V_z2 is %b, expected %b", sim_V, expected_V); //if expected V_z2 does not match actual V_z2, then err = 1
        
        err = 1'b1; // raise error
    
    end

    if (sim_Z !== expected_Z) begin
        $display("ERROR, *** Z_z0 is %b, expected %b", sim_Z, expected_Z); //if expected Z_z0 does not match actual Z_z0, then err = 1
        
        err = 1'b1; // raise error
    
    end

    if (datapath_tb.DUT.data_out !== expected_data_out) begin
        $display("ERROR, *** data_out is %b, expected %b", datapath_tb.DUT.data_out, expected_data_out); //if expected data_out does not match actual data_out, then err = 1
        
        err = 1'b1; // raise error
    
    end

    //  if (datapath_tb.DUT.loada_out !== expected_loada_out) begin
    //     $display("ERROR, *** loada_out is %b, expected %b", datapath_tb.DUT.loada_out, expected_loada_out); //if expected loada_out does not match actual loada_out, then err = 1
        
    //     err = 1'b1; // raise error
    
    // end

    // if (datapath_tb.DUT.loadb_out !== expected_loadb_out) begin
    //     $display("ERROR, *** loadb_out is %b, expected %b", datapath_tb.DUT.loadb_out, expected_loadb_out); //if expected loadb_out does not match actual loadb_out, then err = 1
        
    //     err = 1'b1; // raise error
    
    // end

    // if (datapath_tb.DUT.loadc_out !== expected_loadc_out) begin
    //     $display("ERROR, *** loadc_out is %b, expected %b", datapath_tb.DUT.loadc_out, expected_loadc_out);
        
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

#10; // wait for clk

$display("Cycle 1, Part 1: Storing '7' in R0, checking data_out");
sim_sximm8 = 16'b0000000000000111;                                                                     // datapath_in is given the value of 7
sim_vsel = 2'b01;                                                                                            // vsel is 1 since we want the value of datapath_in
sim_writenum = 3'b000;                                                                                      // value is 0 since we want to store the value in register 0
sim_write = 1'b1;                                                                                           // write is on since we want to write the value past the AND gate to the register
sim_readnum = 3'b000;                                                                                       // readnum is 0 since we want to read the value in register 0
// storing value of R0 in register B (for shifting)
#10; 
output_checker(16'bxxxxxxxxxxxxxxxx,1'bx, 1'bx, 1'bx, 16'b0000000000000111 /*,16'bxxxxxxxxxxxxxxxx,16'bxxxxxxxxxxxxxxxx*/); // everything except for data_out should be undefined in this check. data_out should be 7

$display("Cycle 1, Part 2: Storing '7' in R0, checking data_out");
// storing value into Register B
sim_loada = 1'b0;                                                                                           // loada is 0 and loadb is 1 since we want to load 7 into register B
sim_loadb = 1'b1;
#10; 
output_checker(16'bxxxxxxxxxxxxxxxx, 1'bx, 1'bx, 1'bx, 16'b0000000000000111/*,16'bxxxxxxxxxxxxxxxx,16'b0000000000000111*/); // everything should remain the same as the previous check except for loadb_out since it is now 7

$display("Cycle 2, Part 1: Storing '2' in R1, checking data_out");
sim_sximm8 = 16'b0000000000000010;                                                                     // datapath_in is given the value of 2
sim_vsel = 2'b01;                                                                                            // vsel is 1 since we want the value of datapath_in
sim_writenum = 3'b001;                                                                                      // value is 1 since we want to store the value in register 1
sim_write = 1'b1;                                                                                           // write is on since we want to write the value past the AND gate to the register
sim_readnum = 3'b001;                                                                                       // readnum is 1 since we want to read the value in register 1
// storing value of R1 in register A (for shifting)
sim_loada = 1'b1;                                                                                           // loada is 1 and loadb is 0 since we want to load 2 into register A
sim_loadb = 1'b0;
#10; 
output_checker(16'bxxxxxxxxxxxxxxxx, 1'bx, 1'bx, 1'bx, 16'b0000000000000010/*,16'bxxxxxxxxxxxxxxxx, 16'b0000000000000111*/); // everything should remain the same as the previous check except for data_out which is now 2

$display("Cycle 2, Part 2: Storing '2' in R1, checking data_out");
// storing value into Register A
sim_loada = 1'b1;                                                                                           // loada is 1 and loadb is 0 since we want to load 2 into register A
sim_loadb = 1'b0;
#10; 
output_checker(16'bxxxxxxxxxxxxxxxx, 1'bx, 1'bx, 1'bx, 16'b0000000000000010/*,16'b0000000000000010, 16'b0000000000000111*/); // everything should remain the same as the previous check except for loada_out since it is now 2

$display("Cycle 3: Adding '2' in R1 amd '7' in R0 ---> storing into C
                   ---> setting value to datapath_out");
sim_loada = 1'b0;  // deassert loada

sim_asel = 1'b0;                                                                                            // we want loada_out to pass through MUX

sim_shift = 2'b01; //left shift                                                                             // multiplies the value in register B by 2 (7*2)
sim_bsel = 1'b0;                                                                                            // we want loadb_out to pass through MUX

sim_ALUop = 2'b00; // adding                                                                                // we want to add Ain and Bin

sim_loadc = 1'b1;                                                                                           // loadc is 1 since we want to load the sum into register C

sim_loads = 1'b1;                                                                                           // loads is 1 to check status register

#10; 
output_checker(16'b0000000000010000, 1'b0, 1'b0, 1'b0, 16'b0000000000000010/*,16'b0000000000000010, 16'b0000000000000111*/); // everything should remain the same as the previous check except datapath_out which is now 16 and N,V,Z which is now 0



$display("Cycle 4, Part 1: Storing sum in R2");

sim_loadc = 1'b0;                                                                                            // loadc is set to 0 to deassert

#10; 
output_checker(16'b0000000000010000, 1'b0, 1'b0, 1'b0, 16'b0000000000000010/*,16'b0000000000000010, 16'b0000000000000111*/); // everything should remain the same as the previous check

$display("Cycle 4, Part 2: Storing sum in R2");
// multiplexer "9" chooses datapath_out over datapath_in
sim_vsel = 2'b11;                                                                                             // vsel should be 2 since we want the value of datapath_out to pass the MUX

sim_writenum = 3'b010;                                                                                       // value is 2 since we want to store 16 into register 2
sim_write = 1'b1;                                                                                            // write is on since we want to wrtie the value past the AND gate into the register
sim_readnum = 3'b010;                                                                                        // readnum is 2 since we want to read the value from register 2
#430;
output_checker(16'b0000000000010000, 1'b0, 1'b0, 1'b0, 16'b0000000000010000/*,16'b0000000000000010, 16'b0000000000000111*/); // everything should remain the same as the previous check except for data_out which is now 16


if (~err) $display("PASSED");                                                                                // if err is never 1, then pass
    else $display("FAILED");                                                                                 // if err is ever 1, then fail
    $stop;




end

endmodule



