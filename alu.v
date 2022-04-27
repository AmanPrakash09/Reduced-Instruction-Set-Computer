module ALU(Ain,Bin,ALUop,out,Z);
input [15:0] Ain, Bin;
input [1:0] ALUop;
output [15:0] out;
output [2:0] Z; //is only 1 when out is 0

reg [15:0]out;
reg [2:0] Z;


always @(*) begin //ALUop determines which operation is carried out
    case(ALUop)   //the value of out depends on Ain and Bin
    2'b00: out = Ain + Bin;
    2'b01: out = Ain - Bin;
    2'b10: out = Ain & Bin; // anding will not produce overflow
    2'b11: out = ~Bin;      // will not produce overflow
    default: out = 16'bxxxxxxxxxxxxxxxx;
    endcase

    // range of out: -(2^15) <= out <= (2^15 - 1) 

    // range of out: -(32768) <= out <= (32767) 

    // range of out: -(16'b0111111111111111) <= out <= (16'b1000000000000000)

if (ALUop == 2'b11) begin // cannot have overflow if ~Bin
    Z[2] = 1'b0;
end 
else begin

    if ( ( (Ain[15] == 1'b0) && (Bin[15] == 1'b0) && (out[15] == 1'b1) ) || 
        ( (Ain[15] == 1'b1) && (Bin[15] == 1'b1) && (out[15] == 1'b0) ) ) begin
            Z[2] = 1'b1;
    end 
    else begin
    Z[2] = 1'b0;  
    end
end

    // Note: what happens to other bits of Z in the different cases?
    casex(out)
    16'b0000000000000000: Z[1:0] = 2'b01; // LSB for zero flag // cannot be negative
    16'b1xxxxxxxxxxxxxxx: Z[1:0] = 2'b10; // middle bit for negative flag // cannot be zero
    16'b0xxxxxxxxxxxxxxx: Z[1:0] = 2'b00; // positive numbers

    // N is for Z[1] // negative 
    // V is for Z[2] // overflow
    // Z is for Z[0] // zero
    

    default: Z = 2'bxx;
    endcase


    

// if(out == 16'b0000000000000000) begin
//     Z = 3'b001;                           //if out is 0, then Z is 1
// end
// else begin
//     Z = 1'b0;
// end

end

endmodule