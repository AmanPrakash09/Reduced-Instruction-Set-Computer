
// testbench for LAB5 regfile




module regfile_tb(); // no I/O because testbench

reg [15:0] sim_data_in;
wire [15:0] sim_data_out;  // output is wire
reg [2:0] sim_writenum;
reg [2:0] sim_readnum;
reg sim_write;
reg sim_clk;

reg err; // error check

// INSTANTIATION

regfile DUT(
    .data_in(sim_data_in),
    .writenum(sim_writenum),
    .readnum(sim_readnum),
    .write(sim_write),
    .clk(sim_clk),
    .data_out(sim_data_out)
);

//TASK

task output_checker;
    input expected_load0;
    input expected_load1;
    input expected_load2;
    input expected_load3;
    input expected_load4;
    input expected_load5;
    input expected_load6;
    input expected_load7;
    input [15:0] expected_data_out;
begin
    if (regfile_tb.DUT.load0 !== expected_load0) begin
        $display("ERROR, *** load is %b, expected %b", regfile_tb.DUT.load0, expected_load0);  //if expected load0 does not match actual load0, then err = 1
        
        err = 1'b1; // raise error
    
    end
    if (regfile_tb.DUT.load1 !== expected_load1) begin
        $display("ERROR, *** load is %b, expected %b", regfile_tb.DUT.load1, expected_load1);  //if expected load1 does not match actual load1, then err = 1
        
        err = 1'b1; // raise error
    
    end
    if (regfile_tb.DUT.load2 !== expected_load2) begin
        $display("ERROR, *** load is %b, expected %b", regfile_tb.DUT.load2, expected_load2);  //if expected load2 does not match actual load2, then err = 1
        
        err = 1'b1; // raise error
    
    end
    if (regfile_tb.DUT.load3 !== expected_load3) begin
        $display("ERROR, *** load is %b, expected %b", regfile_tb.DUT.load3, expected_load3);  //if expected load3 does not match actual load3, then err = 1
    
        err = 1'b1; // raise error
    
    end
    if (regfile_tb.DUT.load4 !== expected_load4) begin
        $display("ERROR, *** load is %b, expected %b", regfile_tb.DUT.load4, expected_load4);  //if expected load4 does not match actual load4, then err = 1
        
        err = 1'b1; // raise error
    
    end
    if (regfile_tb.DUT.load5 !== expected_load5) begin
        $display("ERROR, *** load is %b, expected %b", regfile_tb.DUT.load5, expected_load5);  //if expected load5 does not match actual load5, then err = 1
        
        err = 1'b1; // raise error
    
    end
    if (regfile_tb.DUT.load6 !== expected_load6) begin
        $display("ERROR, *** load is %b, expected %b", regfile_tb.DUT.load6, expected_load6);  //if expected load6 does not match actual load6, then err = 1
        
        err = 1'b1; // raise error
    
    end
    if (regfile_tb.DUT.load7 !== expected_load7) begin
        $display("ERROR, *** load is %b, expected %b", regfile_tb.DUT.load7, expected_load7);  //if expected load7 does not match actual load7, then err = 1
        
        err = 1'b1; // raise error
    
    end

    if (sim_data_out !== expected_data_out) begin
        $display("ERROR, *** output is %b, expected %b", sim_data_out, expected_data_out);  //if expected data_out does not match actual data_out, then err = 1

        err = 1'b1; // raise error
    end
        
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

#125; // wait for clk


$display("Checking Write to R1");                                               //putting 2 into register 1 and reading from it
sim_data_in = 16'b0000000000000010;                                             // 2
sim_writenum = 3'b001;                                                          // writenum is 1 to wrtie the value into register 1
sim_write = 1'b1;                                                               // write is on since we want to write the value past the AND gate to the register
sim_readnum = 3'b001;                                                           // readnum is 1 since we want to read the value in register 1
#125; 
output_checker(1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,16'b0000000000000010);   // all the loads should be 0 except for load1 since we are writenum will lead to a onehot of 8'b00000010. data_out should be 2

$display("Checking Write to R1");                                               //putting 3 into register 1 and reading from it BUT write is 0 -----> nothing data_out should not change and all loads should be off (0)
sim_data_in = 16'b0000000000000011;                                             // 3
sim_writenum = 3'b001;                                                          // writenum is 1 to wrtie the value into register 1
sim_write = 1'b0;                                                               // write is off since we do not want to write the value past the AND gate to the register
sim_readnum = 3'b001;                                                           // readnum is 1 since we want to read the value in register 1
#125; 
output_checker(1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,16'b0000000000000010);   // all the loads should be 0. data_out should be 2

$display("Checking Write to R1");                                               //putting 3 into register 1 and reading from it
sim_data_in = 16'b0000000000000011;                                             // 3
sim_writenum = 3'b001;                                                          // writenum is 1 to wrtie the value into register 1
sim_write = 1'b1;                                                               // write is on since we want to write the value past the AND gate to the register
sim_readnum = 3'b001;                                                           // readnum is 1 since we want to read the value in register 1
#125; 
output_checker(1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,16'b0000000000000011);   // all the loads should be 0 except for load1 since we are writenum will lead to a onehot of 8'b00000010. data_out should be 3


if (~err) $display("PASSED");                                                   // if err is never 1, then pass
    else $display("FAILED");                                                    // if err is ever 1, then fail
    $stop;




end

endmodule



