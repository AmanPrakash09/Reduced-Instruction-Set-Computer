
// testbench for  LAB5 shifter

module shifter_tb();  // no IO as this is a testbench

reg [15:0] sim_in;
reg [1:0] sim_shift;
// outputs are wires
wire [15:0] sim_sout;


reg err; // error check

// instantiation

shifter DUT(
    .in(sim_in),
    .shift(sim_shift),
    .sout(sim_sout)
   
);

task output_checker; // input the expected output into the function
    input [15:0] expected_sout;
begin
    // check if we are in the expected out state
    if (sim_sout !== expected_sout) begin
        $display("ERROR, *** output is %b, expected %b", sim_sout, expected_sout);  //if expected sout does not match actual sout, then err = 1

        err = 1'b1; // raise error
    end
    
end
endtask


initial begin

    // set error to 0
    err = 1'b0;

    $display("Checking no shift");                                          // shifter should do nothing
    sim_in = 16'b1111000011001111;                                          // 61647
    sim_shift = 2'b00;                                                      // do absolutely nothing
    #125; // allow delay for changes to happen
    output_checker(16'b1111000011001111);                                   // 61647

    $display("Checking B shifted left 1-bit, LSB is 0");                    // shifter should shift to the left
    sim_in = 16'b1111000011001111;                                          // 61647
    sim_shift = 2'b01;                                                      // move one bit to the left
    #125; // allow delay for changes to happen
    output_checker(16'b1110000110011110);                                   // 57758

    $display("Checking B shifted right 1-bit, MSB is 0");                   // shifter should shift to the right
    sim_in = 16'b1111000011001111;                                          // 61647
    sim_shift = 2'b10;                                                      // move one bit to the right
    #125; // allow delay for changes to happen
    output_checker(16'b0111100001100111);                                   // 30823

    $display("Checking B shifted right 1-bit, MSB is copy of B[15]");       // shifter should shift to the right while MSB is copy of B[15]
    sim_in = 16'b1111000011001111;                                          // 61647
    sim_shift = 2'b11;                                                      // move one bit to the right while MSB is copy of B[15]
    #125; // allow delay for changes to happen
    output_checker(16'b1111100001100111);                                   // 63591

    
    if (~err) $display("PASSED");                                           // if err is never 1, then pass
    else $display("FAILED");                                                // if err is ever 1, then fail

end

endmodule