module shifter(in,shift,sout);
input [15:0] in;
input [1:0] shift;
output [15:0] sout;

reg [15:0]sout;

always @(*) begin //the value of shift determines the shift operation
    case(shift)
    2'b00: sout = in;
    2'b01: sout = in << 1;
    2'b10: sout = in >> 1;
    2'b11: begin
        {sout[0], sout[1], sout[2], sout[3], sout[4], sout[5], sout[6], sout[7],
        sout[8], sout[9], sout[10], sout[11], sout[12], sout[13], sout[14], sout[15]}
        =
        {in[1], in[2], in[3], in[4], in[5], in[6], in[7], in[8], in[9], in[10],
        in[11], in[12], in[13], in[14], in[15], in[15]};
    end
    default: sout = 16'bxxxxxxxxxxxxxxxx;
    
    endcase
end

endmodule