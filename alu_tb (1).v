
// testbench for  LAB5 alu

module ALU_tb();  // no IO as this is a testbench

reg [15:0] sim_Ain;
reg [15:0] sim_Bin;
reg [1:0] sim_ALUop;

// outputs are wires
wire [15:0] sim_out;
wire [2:0] sim_Z;

reg err; // error check

// instantiation

ALU DUT(
    .Ain(sim_Ain),
    .Bin(sim_Bin),
    .ALUop(sim_ALUop),
    .out(sim_out),
    .Z(sim_Z)

);

task output_checker; // input the expected output into the function
    input [15:0] expected_out;
    input [2:0] expected_Z;

begin
    // check if we are in the expected out state
    if (sim_out !== expected_out) begin
        $display("ERROR, *** output is %b, expected %b", sim_out, expected_out); //if expected out does not match actual out, then err = 1

        err = 1'b1; // raise error
    end
    if (sim_Z !== expected_Z) begin
        $display("ERROR, *** output is %b, expected %b", sim_Z, expected_Z);  //if expected Z does not match actual Z, then err = 1

        err = 1'b1; // raise error
    end
end
endtask


initial begin

    // set error to 0
    err = 1'b0;

    $display("Checking addition");              // checking 1 + 3 = 4
    sim_Ain = 16'b0000000000000001;             // 1
    sim_Bin = 16'b0000000000000011;             // 3
    sim_ALUop = 2'b00;                          // +
    #100;                                       // allow delay for changes to happen
    output_checker(16'b0000000000000100,3'b000);  // out is 4, Z is 0 (bc out is not 0)

    $display("Checking subtraction");           // checking 3 - 1 = 2
    sim_Ain = 16'b0000000000000011;             // 3
    sim_Bin = 16'b0000000000000001;             // 1
    sim_ALUop = 2'b01;                          // -
    #100;                                       // allow delay for changes to happen
    output_checker(16'b0000000000000010,3'b000);  // out is 2, Z is 0 (bc out is not 0)

    $display("Checking ANDING!!!!! ");          // checking 3 & 1 = 1
    sim_Ain = 16'b0000000000000011;             // 3            
    sim_Bin = 16'b0000000000000001;             // 1
    sim_ALUop = 2'b10;                          // &
    #100;                                       // allow delay for changes to happen
    output_checker(16'b0000000000000001,3'b000);  // out is 1, Z is 0 (bc out is not 0)

    $display("Checking NOT Bin, out is -ve");     // checking ! 1  
    sim_Ain = 16'b0000000000000011;             // 3            
    sim_Bin = 16'b0000000000000001;             // 1
    sim_ALUop = 2'b11;                          // !
    #100;                                       // allow delay for changes to happen
    output_checker(16'b1111111111111110,3'b010);  // out is opposite of 1, Z is 010 (bc out is -ve, does not match defenition of overflow)

    $display("Checking Z = 001");                 // case to see if Z will turn to 001 when out will be 0
    sim_Ain = 16'b0000000000000001;             // 1
    sim_Bin = 16'b0000000000000001;             // 1
    sim_ALUop = 2'b01;                          // -
    #100;                                       // allow delay for changes to happen
    output_checker(16'b0000000000000000,3'b001);  // out is 0, Z is 001 (bc out is 0)

    $display("Checking Z = 110");                 // case to see if Z will turn to 010 when out will be 0
    sim_Ain = 16'b0000000000000000;             // 0
    sim_Bin = 16'b0000000000000001;             // 1
    sim_ALUop = 2'b01;                          // -
    #100;                                       // allow delay for changes to happen
    output_checker(16'b1111111111111111,3'b110);  // out is -1, Z is 110 (bc out is -ve)

    $display("Checking Z = 101");                 // case to see if Z will turn to 100 when out will be 
    sim_Ain = 16'b1000000000000000;             // -32768
    sim_Bin = 16'b1000000000000000;             // -32768
    sim_ALUop = 2'b00;                          // +
    #100;                                       // allow delay for changes to happen
    output_checker(16'b0000000000000000,3'b101);  // out is -65536 but only 16 bits -> so represents 0, Z is 101 (bc out is 0 and has overflow)

    $display("Checking Z = 110");                 // case to see if Z will turn to 100 when out will be 
    sim_Ain = 16'b0111111111111111;             // 32767
    sim_Bin = 16'b0111111111111111;             // 32767
    sim_ALUop = 2'b00;                          // +
    #100;                                       // allow delay for changes to happen
    output_checker(16'b1111111111111110,3'b110);  // out is 65534 but MSB is 1 -> so represents -2, Z is 110 (bc out is -ve and has overflow)

    $display("Checking ANDING, out is 0 ");          // checking -1 & 0 = 0
    sim_Ain = 16'b1111111111111111;             // -1            
    sim_Bin = 16'b0000000000000000;             // 0
    sim_ALUop = 2'b10;                          // &
    #100;                                       // allow delay for changes to happen
    output_checker(16'b0000000000000000,3'b001);  // out is 0, Z is 001 (bc out is 0)
    
    if (~err) $display("PASSED");               // if err is never 1, then pass
    else $display("FAILED");                    // if err is ever 1, then fail

end

endmodule