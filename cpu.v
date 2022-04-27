
// OLD state encoding

// `define Wait 3'b000
// `define Decode 3'b001
// `define GetA 3'b010
// `define GetB 3'b011
// `define ExecuteOperation 3'b100
// `define WriteReg 3'b101
// `define Writelmm 3'b110

// NEW STATE ENCODING

// `define Wait 4'b0000

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


module cpu(clk,reset,read_data,write_data,mem_addr,mem_cmd,N,V,Z);
input clk, reset;
input [15:0] read_data; // 16 bits wide
output [15:0] write_data; // 16 bits wide
output [8:0] mem_addr; // 9 bits wide
output [1:0] mem_cmd; // 2 bits wide
output N, V, Z;


wire load_ir; // wire from SM to IR

wire [15:0] IR_out; // wire coming out of instruction register

// Instruction Resister
registerCPU #(16) IR(read_data, load_ir, clk, IR_out);

wire [1:0] nsel; // determins Rn, Rd, Rm // from Controller FSM
wire [2:0] opcode; // to Controller FSM [15:13]
wire [1:0] op; // to Controller FSM [12:11]
wire [2:0] Rn; // register [10:8]
wire [2:0] Rd; // register [7:5]
wire [2:0] Rm; // register [2:0]
wire [2:0] readnum; // comes out of Mux31
wire [2:0] writenum; // comes out of Mux31
wire [1:0] shift; // shift [4:3]
wire [7:0] imm8; // [7:0]
wire [15:0] sximm8; // after sign extend
wire [4:0] imm5; // [4:0]
wire [15:0] sximm5; // after sign extend
wire [1:0] ALUop; // [12:11]

// Instruction Decoder
InstructionDecoder ID(IR_out, nsel, opcode, op, Rn, Rd, Rm, readnum, writenum,
                        shift, imm8, sximm8, imm5, sximm5, ALUop); // Dis good

// wires for StateMachine

wire loada, loadb, loadc, loads, write; // wires connecting from statemachine to datapath

wire [1:0] vsel; // 2 bits wide, connecting statemachine to datapath

wire asel, bsel; // wires connecting from statemachine to datapath

wire addr_sel, load_pc, reset_pc, load_addr;

StateMachine SM(opcode, op, mem_cmd, addr_sel, load_pc, load_ir, reset_pc, load_addr,
                clk, reset, loada, loadb, loadc, loads, asel, bsel, vsel, write, nsel);

// wire [15:0] mdata; // equals read_data input from cpu

wire [8:0] PC; // wire from Program Counter to Mux21 and right most 8 bits to datapath

datapath DP(read_data, sximm8, /*PC,*/ {8'b00000000,PC[7:0]},
                writenum, write, readnum,clk,
                loada, loadb, loadc, loads,
                asel, bsel, sximm5, vsel,
                ALUop,
                shift,
                N,
                V,
                Z,
                write_data); // datapath_out = write_data

// comments for DP I/O's added in datapath.v

wire [8:0] DA_out; // wire from Data Address to Mux21

// instantiation for DataAddress. Holds ??? // NEED TO COMMENT
registerCPU #(9) DataAddress(write_data[8:0], load_addr, clk, DA_out);

Mux21CPU AS(PC, DA_out, addr_sel, mem_addr);

wire [8:0] PC_plus_one; 
assign PC_plus_one = 9'b000000001 + PC; // incrementing PC by one

wire [8:0] next_ProgramCounter; // wire from Mux21 to Program Counter

Mux21CPU RPC(9'b000000000, PC_plus_one, reset_pc, next_ProgramCounter);

registerCPU #(9) ProgramCounter(next_ProgramCounter, load_pc, clk, PC); 


endmodule


module InstructionDecoder(IR_out, nsel, opcode, op, Rn, Rd, Rm, readnum, writenum,
                         shift, imm8, sximm8, imm5, sximm5, ALUop);

input [15:0] IR_out; // wire that comes out of instruction register
input [1:0] nsel; // determins Rn, Rd, Rm // from Controller FSM

output reg[2:0] opcode; // to Controller FSM [15:13]
output reg[1:0] op; // to Controller FSM [12:11]

output reg [2:0] Rn; // register [10:8]
output reg [2:0] Rd; // register [7:5]
output reg [2:0] Rm; // register [2:0]

output [2:0] readnum; // comes out of Mux31
output [2:0] writenum; // comes out of Mux31
output reg [1:0] shift; // shift [4:3]

output reg [7:0] imm8; // [7:0]
output [15:0] sximm8; // after sign extend

output reg [4:0] imm5; // [4:0]
output [15:0] sximm5; // after sign extend

output reg [1:0] ALUop; // [12:11]

// assigning wires 

always @(*) begin // assigned base on bits of IR_out
    
 opcode = IR_out[15:13];
 op = IR_out[12:11];
 Rm = IR_out[2:0];
 Rd = IR_out[7:5];
 Rn = IR_out[10:8];
 shift = IR_out[4:3];
 imm8 = IR_out[7:0];
 imm5 = IR_out[4:0];
 ALUop = IR_out[12:11];

end

Mux31 RegSel(Rm, Rd, Rn, nsel, readnum, writenum); // Determines which register to read/write from
SignExtend_8 SE8(imm8, sximm8); // adds bits to 16
SignExtend_5 SE5(imm5, sximm5); // adds bits to 16

endmodule

module Mux31(a, b, c, selector, r_out, w_out);
input [2:0] a, b, c; // all are 3 bits wide
input [1:0] selector; // need 2 bits
output reg [2:0] r_out; // 3 bits wide 
output reg [2:0] w_out; // 3 bits wide 

always @(*) begin 
    case(selector)
    2'b00: begin
        r_out = a; // if selector is 0, out is a
        w_out = a;
    end
    2'b01: begin 
        r_out = b; // if selector is 1, out is b
        w_out = b;
    end
    2'b10: begin 
        r_out = c; // if selector is 2, out is c
        w_out = c;
    end
    default: begin 
        r_out = 3'bxxx;
        w_out = 3'bxxx;
    end
        
    
    endcase
end
endmodule


module SignExtend_8(tiny, big);
input [7:0] tiny;
output reg [15:0] big;

always @(*) begin
    casex(tiny)
    8'b0xxxxxxx: big = {8'b0, tiny}; // if positive, concatenating 8 zeros to the left
    8'b1xxxxxxx: big = {8'b1, tiny}; // if negative, concatenating 8 ones to the left
    default: big = 16'bxxxxxxxxxxxxxxxx;
    endcase 
end

endmodule

module SignExtend_5(tiny, big);
input [4:0] tiny;
output reg [15:0] big;

always @(*) begin
    casex(tiny)
    5'b0xxxx: big = {11'b0, tiny}; // if positive, concatenating 11 zeros to the left
    5'b1xxxx: big = {11'b1, tiny}; // if negative, concatenating 11 ones to the left
    default: big = 16'bxxxxxxxxxxxxxxxx;
    endcase 
end

endmodule

// STATE MACHINE MODULE
module StateMachine (opcode, op, mem_cmd, addr_sel, load_pc, load_ir, reset_pc, load_addr,
                clk, reset, loada, loadb, loadc, loads, asel, bsel, vsel, write, nsel);

    input [2:0] opcode; // to Controller FSM [15:13]
    input [1:0] op; // to Controller FSM [12:11]
    output [1:0] mem_cmd;
    output addr_sel, load_pc, load_ir, reset_pc, load_addr;
    input clk;
    input reset;
    output loada, loadb, loadc, loads; 
    output write;
    output [1:0] vsel; // 2 bits wide
    output [1:0] nsel; // 2 bits wide
    output asel, bsel;

    wire [4:0] state; // output of moore machine

    // instanting modules

    StateMachine_mooremachine mooremachine(clk,reset,state,opcode,op); // sequential logic for states
    StateMachine_combinational combinational(state, mem_cmd, addr_sel, load_pc, load_ir, reset_pc, load_addr,
                                            loada, loadb, loadc, loads, asel, bsel, vsel, write, nsel); // combinational logic to determine output wires


endmodule

module StateMachine_mooremachine(clk,reset,state,opcode,op);

// Need to declare the wires
    input clk;
    input reset;

    // input s; // for state transition from Wait to Decode
    input [2:0] opcode; // to Controller FSM [15:13]
    input [1:0] op; // to Controller FSM [12:11]

    output [4:0] state;

    // reg declarations
    reg [4:0] present_state; // 4-bit wide being modified in case statements
    reg [4:0] state;         // 4-bit wide being modified at the end


    always @(posedge clk) begin // sequential logic
        
        if (reset) begin
            // present_state = `Wait;
            present_state = `RST;
        end else begin
            case(present_state)
                `RST: begin
                    // if (s == 1'b1)
                    present_state = `IF1;
                end
                `IF1: begin
                    present_state = `IF2;
                end
                `IF2: begin
                    present_state = `UpdatePC;
                end
                `UpdatePC: begin
                    present_state = `Decode;
                  
                end


                // OLD DECODE STATE CASE
                // `Decode: begin 
                //     if (opcode == 3'b110)
                //     present_state = `Writelmm; // transition to Writelmm for MOV Rn#<im8>
                //     else if (opcode == 3'b101)
                //     present_state = `GetA;    // transition to GetA
                //     else 
                //     present_state = `Decode; // invalid opcode stays as same state
                // end

                `Decode: begin 
                    // MOVE INSTRUCTIONS
                    if (opcode == 3'b110 && op == 2'b10)
                    present_state = `Writelmm; // transition to Writelmm for MOV Rn#<im8>
                    else if (opcode == 3'b110 && op == 2'b00)
                    present_state = `GetA; // transition to GetA for MOV Rd,Rm

                    else if (opcode == 3'b011 && op == 2'b00)
                    present_state = `GetA; // transition to GetA (for LDR)
                    else if (opcode == 3'b100 && op == 2'b00)
                    present_state = `GetA; // transition to GetA (for STR)
                    else if (opcode == 3'b111 && op == 2'b00)
                    present_state = `HALT; // transition to HALT and STAY THERE. assert reset manualy to go back to RST


                    // ALU INSTSRUCTIONS (will always go to GetA. But will branch at GetB depending on operation)
                    else if (opcode == 3'b101)
                    present_state = `GetA;    // transition to GetA for ALU operations
                    else 
                    present_state = `Decode; // invalid opcode stays as same state
                end

                // Writelmm branch 
                `Writelmm: present_state = `IF1;  // Always True

/*
                // OLD ALU BRANCH STATE
                // ALU BRANCH
                `GetA: present_state = `GetB; // always true

                `GetB: present_state = `ExecuteOperation; // always true

                `ExecuteOperation: present_state = `WriteReg; 
               
                `WriteReg: present_state = `Wait; // always true
*/            


                // ALU BRANCH + (MOV rd,Rm) !!!!!! NEED TO MAKE NEW STATES  AFTER GETB !!!!!
                // NEED TO ADD NEW STATES: `MOV_Rd_Rm,  `ADD ... 
                // WILL HAVE TO EXTEND STATE ENCODING TO 4 BITS FOR THE 11 STATES
                `GetA: begin
                    if (opcode == 3'b011 && op == 2'b00)
                    present_state = `LDR; // transition to LDR for LDR Rd,[Rn{,#<im5>}]
                    else if (opcode == 3'b100 && op == 2'b00)
                    present_state = `GetRd; // transition to STR for STR Rd,[Rn{,#<im5>}]
                    else
                    present_state = `GetB; // We are not Loading or Storing
                end
                `GetB: begin
                    if (opcode == 3'b110 && op == 2'b00)
                    present_state = `MOV_Rd_Rm; // transition to MOV_Rd_Rm for MOV Rd,Rm
                    else if (opcode == 3'b101 && op == 2'b00)
                    present_state = `ADD; // transition to ADD for ADD Rd,Rn,Rm
                    else if (opcode == 3'b101 && op == 2'b01)
                    present_state = `CMP; // transition to CMP for CMP Rn,Rm
                    else if (opcode == 3'b101 && op == 2'b10)
                    present_state = `AND; // transition to AND for AND Rd,Rn,Rm
                    else if (opcode == 3'b101 && op == 2'b11)
                    present_state = `MVN; // transition to MVN for MVN Rd,Rm  
                    else 
                    present_state = `HALT; // If wrong Opcode / op, goes to HALT state. wait for reset    
                end

                // State transitions after branching from Get B (Previously `ExecuteOperation: present_state = `WriteReg;)
                `MOV_Rd_Rm: present_state = `WriteReg; 

                `ADD: present_state = `WriteReg; 

                `CMP: present_state = `IF1; // Not writing to the register 

                `AND: present_state = `WriteReg; 

                `MVN: present_state = `WriteReg; 

                // writing from memory to register
                `LDR: present_state = `LDR_MEM_DA; // LDR branch for loading R[rn] + sx(im5) into Data Address register

                `LDR_MEM_DA: present_state = `LDR_MEM_MDATA; // LDR branch for setting mem_cmd to MREAD and addr_sel to 0 so that mdata gets value from memory

                `LDR_MEM_MDATA: present_state = `LDR_MEM_Rd; // LDR branch for writing M [ R[rn] + sx(im5) ] into Rd

                `LDR_MEM_Rd: present_state = `IF1; // ending branch for LDR instruction

                // writing from register to memory (STR)
                `GetRd: present_state = `STR; // Go to STR state

                `STR: present_state = `STR_MEM_DA; // STR branch for loading R[rn] + sx(im5) into Data Address register

                `STR_MEM_DA: present_state = `STR_MEM_DIN; // STR branch for keeping R[rn] + sx(im5) into Data Address register and changing datapath_out
                
                `STR_MEM_DIN: present_state = `STR_MEM_RAM; // STR branch for loading Rd into address R[rn] + sx(im5) into RAM

                `STR_MEM_RAM: present_state = `IF1; // ending branch for STR instruction


                // transition from writereg state
                `WriteReg: present_state = `IF1; // always true

                `HALT: present_state = `HALT; // always true

                default: present_state = 5'bxxxxx;

            endcase           
    
        end

        state = present_state; // update the output wire 

    end

endmodule

module StateMachine_combinational(state, mem_cmd, addr_sel, load_pc, load_ir, reset_pc, load_addr,
                                loada, loadb, loadc, loads, asel, bsel, vsel, write, nsel);

    // here we check for what state we are in to output corresponding output value
    input [4:0] state;

    output reg [1:0] mem_cmd;
    output reg addr_sel, load_pc, load_ir, reset_pc, load_addr;
    output reg loada, loadb, loadc, loads; 
    output reg write;
    output reg [1:0] vsel; // 2 bits wide
    output reg [1:0] nsel; // 2 bits wide
    output reg asel, bsel;

    always @(*) begin
        
        case(state)

        // RST state
        `RST: begin 
            nsel = 2'b00; // deassert nsel
            vsel = 2'b00; // deassert vsel
            write = 1'b0; // deassert write
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b0; // deassert
            loads = 1'b0; // deassert
            asel = 1'b1; // default 16'b0
            bsel = 1'b1; // sximm5

            reset_pc = 1'b1; // to input zero into the program counter register
            load_pc = 1'b1; // to load the zero value into the program counter register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data
            
            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 
           
        end   
             
        // IF1 state
        `IF1: begin 
            nsel = 2'b00; // deassert nsel
            vsel = 2'b00; // deassert vsel
            write = 1'b0; // deassert write
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b0; // deassert
            loads = 1'b0; // deassert
            asel = 1'b1; // default 16'b0
            bsel = 1'b1; // sximm5

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register
            
            mem_cmd = `MREAD; // reading 
            addr_sel = 1'b1; // mem_addr is PC
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address.


            // if (opcode == 3'b011 && op == 2'b00) begin
            //   mem_cmd = `MWRITE;  
            // end
            // else begin
            // end

        end        

        `IF2: begin 
            nsel = 2'b00; // deassert nsel
            vsel = 2'b00; // deassert vsel
            write = 1'b0; // deassert write
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b0; // deassert
            loads = 1'b0; // deassert
            asel = 1'b1; // default 16'b0
            bsel = 1'b1; // sximm5

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MREAD; // reading
            addr_sel = 1'b1; // mem_addr is PC
            load_ir = 1'b1; // loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address.    
            
        end 

        `UpdatePC: begin 
            nsel = 2'b00; // deassert nsel
            vsel = 2'b00; // deassert vsel
            write = 1'b0; // deassert write
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b0; // deassert
            loads = 1'b0; // deassert
            asel = 1'b1; // default 16'b0
            bsel = 1'b1; // sximm5

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b1; // to load the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 


        
            
        end               

        // Decode state
        `Decode: begin
            nsel = 2'b00; // deassert nsel
            vsel = 2'b01; // deassert vsel
            write = 1'b0; // not writing in this state
            loada = 1'b0; // not loading in this state
            loadb = 1'b0; // not loading in this state
            loadc = 1'b0; // not loading in this state
            loads = 1'b0; // not loading in this state
            asel = 1'b1; // default 16'b0
            bsel = 1'b1; // sximm5

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 

        end

        // *** MOV BRANCH ***
        `Writelmm: begin
            asel = 1'b1; // default 16'b0
            bsel = 1'b1; // sximm5
            loada = 1'b0; // not loading so deassert
            loadb = 1'b0; // not loading so deassert
            loadc = 1'b0; // not writing from datapath_out. writing from immdiate
            loads = 1'b0; // not checking status

            nsel = 2'b10; // To choose Rn from Mux31 (for loading), need nsel = 2
            vsel = 2'b01; // vsel = 1 to select sximm8
            write = 1'b1; // enable write to register

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 


        end

        // *** ALU BRANCH + MOV_Rd_Rm BRANCH *** 

        // Executing Operation States (ADD/AND Rd, Rn, Rm) -> need to load Rn and Rm, then store in Rd
        `GetA: begin
            write = 1'b0; // deassert as we are loading b
            asel = 1'b0;  // doesent matter as we are not loading c
            bsel = 1'b0;  // doesent matter as we are not loading c
            vsel = 2'b01; // doesent matter as write is off
            loadb = 1'b0; // deassert
            loadc = 1'b0; // deassert
            loads = 1'b0; // deassert

            nsel = 2'b10; // To choose Rn from Mux31, need nsel = 2
            loada = 1'b1; // loading into a

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register
            
            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 


        end

        `GetB: begin 
            write = 1'b0; // deassert as we are loading b
            asel = 1'b0;  // doesent matter as we are not loading c
            bsel = 1'b0;  // doesent matter as we are not loading c
            vsel = 2'b01; // doesent matter as write is off
            loadc = 1'b0; // deassert
            loads = 1'b0; // deassert
            loada = 1'b0; // deassert

            nsel = 2'b00; // To choose Rm from Mux31, need nsel = 0
            loadb = 1'b1; // loading into b

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            load_addr = 1'b0; // NO enable loading into Data Address. 
        end

        // storing Rd into Register B
        `GetRd: begin 
            write = 1'b0; // deassert as we are loading b
            asel = 1'b0;  // doesent matter as we are not loading c
            bsel = 1'b0;  // doesent matter as we are not loading c
            vsel = 2'b01; // doesent matter as write is off
            loadc = 1'b0; // deassert
            loads = 1'b0; // deassert
            loada = 1'b0; // deassert

            nsel = 2'b01; // To choose Rd from Mux31, need nsel = 1
            loadb = 1'b1; // loading into b

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 
        end

        // State transitions from GetB // NEW !!
        
        // *** MOV BRANCH ***
        `MOV_Rd_Rm: begin
            loada = 1'b0; // deassert to prevent loading
            loadb = 1'b0; // deassert to prevent loading
            write = 1'b0; // deassert
            vsel = 2'b01; // doesnt matter as write is off
            nsel = 2'b00; // doesnt matter as load a/b is off
        
            asel = 1'b1; // we need to add 0 in order to get B into C after adding
            bsel = 1'b0; // Bin = Rm
            loadc = 1'b1; // store into c
            loads = 1'b0; // not checking status

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 
        end
        // *** ALU BRANCH ***
        `ADD: begin
            loada = 1'b0; // deassert to prevent loading
            loadb = 1'b0; // deassert to prevent loading
            write = 1'b0; // deassert
            vsel = 2'b01; // doesnt matter as write is off
            nsel = 2'b00; // doesnt matter as load a/b is off
        
            asel = 1'b0; // Need both A and B 
            bsel = 1'b0; // Need both A and B 
            loadc = 1'b1; // store into c
            loads = 1'b0; // not checking status

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 
        end 
        // *** ALU BRANCH ***
        `CMP: begin
            loada = 1'b0; // deassert to prevent loading
            loadb = 1'b0; // deassert to prevent loading
            write = 1'b0; // deassert
            vsel = 2'b01; // doesnt matter as write is off
            nsel = 2'b00; // doesnt matter as load a/b is off

            asel = 1'b0; // Need both A and B 
            bsel = 1'b0; // Need both A and B 
            loadc = 1'b0; // WE DO NOT STORE INTO REGISTER FOR CMP
            loads = 1'b1; // WE ARE CHECKING STATUS

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 
        end
        // *** ALU BRANCH ***
        `AND: begin
            loada = 1'b0; // deassert to prevent loading
            loadb = 1'b0; // deassert to prevent loading
            write = 1'b0; // deassert
            vsel = 2'b01; // doesnt matter as write is off
            nsel = 2'b00; // doesnt matter as load a/b is off
        
            asel = 1'b0; // Need both A and B 
            bsel = 1'b0; // Need both A and B 
            loadc = 1'b1; // store into c
            loads = 1'b0; // not checking status

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 
        end 
        // *** ALU BRANCH ***
        `MVN: begin
            loada = 1'b0; // deassert to prevent loading
            loadb = 1'b0; // deassert to prevent loading
            write = 1'b0; // deassert
            vsel = 2'b01; // doesnt matter as write is off
            nsel = 2'b00; // doesnt matter as load a/b is off
        
            asel = 1'b1; // Do not need A
            bsel = 1'b0; // Only need  B 
            loadc = 1'b1; // store into c
            loads = 1'b0; // not checking status

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 
        end

        `LDR: begin
            nsel = 2'b10; // doesn't matter as we have already have Rn in register A from GetA
            vsel = 2'b01; // doesn't matter as we are not writing
            write = 1'b0; // deassert write
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b1; // store R[Rn]+sx(im5) into c
            loads = 1'b0; // deassert
            asel = 1'b0; // get value from Register A
            bsel = 1'b1; // get sximm5

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // not reading from memory yet
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 
        end

        `LDR_MEM_DA: begin 
            nsel = 2'b10; // doesn't matter as we have already have Rn in register A from GetA
            vsel = 2'b01; // doesn't matter as we are not writing
            write = 1'b0; // deassert write
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b0; // not loading
            loads = 1'b0; // deassert
            asel = 1'b0; // get value from Register A 
            bsel = 1'b0; // doesnt matter

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // not reading from memory yet
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            load_addr = 1'b1; // loading R[rn] + sx(im5) into Data Address
        end

        `LDR_MEM_MDATA: begin 
            nsel = 2'b10; // doesn't matter as we have already have Rn in register A from GetA
            vsel = 2'b01; // doesn't matter as we are not writing
            write = 1'b0; // deassert write
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b0; // not loading
            loads = 1'b0; // deassert
            asel = 1'b0; // get value from Register A 
            bsel = 1'b0; // doesnt matter

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MREAD; // reading from memory to get M [ R[rn] + sx(im5) ] into mdata
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            load_addr = 1'b0; // keep R[rn] + sx(im5) in Data Address
        end

        `LDR_MEM_Rd: begin 
            nsel = 2'b01; // writing to register Rd
            vsel = 2'b00; // writing from mdata
            write = 1'b1; // writing to register Rd
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b0; // not loading
            loads = 1'b0; // deassert
            asel = 1'b0; // get value from Register A 
            bsel = 1'b0; // doesnt matter

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MREAD; // reading from memory to get M [ R[rn] + sx(im5) ] into mdata
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            load_addr = 1'b0; // keep R[rn] + sx(im5) in Data Address
        end
        

        `STR: begin
            nsel = 2'b01; // doesn't matter as we have already have Rd in register B from GetRd
            vsel = 2'b01; // doesn't matter as we are not writing to register
            write = 1'b0; // deassert write
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b1; // store R[Rn]+sx(im5) into c
            loads = 1'b0; // deassert
            asel = 1'b0; // trying to get value from Register A
            bsel = 1'b1; // sximm5

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing YET
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            load_addr = 1'b0; // NO enable loading into Data Address. 
        end

        `STR_MEM_DA: begin
            nsel = 2'b01; // doesn't matter as we have already have Rd in register B from GetRd
            vsel = 2'b01; // doesn't matter as we are not writing to register
            write = 1'b0; // deassert write
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b0; // deassert
            loads = 1'b0; // deassert
            asel = 1'b0; // trying to get value from Register A
            bsel = 1'b1; // sximm5

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing YET
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            load_addr = 1'b1; // loading R[rn] + sx(im5) into Data Address
        end

        `STR_MEM_DIN: begin
            nsel = 2'b01; // doesn't matter as we have already have Rd in register B from GetRd
            vsel = 2'b01; // doesn't matter as we are not writing to register
            write = 1'b0; // deassert write
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b1; // load Rd into Register C
            loads = 1'b0; // deassert
            asel = 1'b1; // Ain = 0 because we only want Rd
            bsel = 1'b0; // selecting Rd from register B

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing YET (need Din in next state)
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            load_addr = 1'b0; // keep R[rn] + sx(im5) in Data Address
        end

        `STR_MEM_RAM: begin
            nsel = 2'b01; // doesn't matter as we have already have Rd in register B from GetRd
            vsel = 2'b01; // doesn't matter as we are not writing to register
            write = 1'b0; // deassert write
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            loadc = 1'b0; // deassert
            loads = 1'b0; // deassert
            asel = 1'b1; // Ain = 0 because we only want Rd
            bsel = 1'b0; // selecting Rd from register B

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MWRITE; // Not reading or writing YET (need Din in next state)
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            load_addr = 1'b0; // keep R[rn] + sx(im5) in Data Address
        end



        // OLD ExecuteOperation state
        // `ExecuteOperation: begin
        //     loada = 1'b0; // deassert to prevent loading
        //     loadb = 1'b0; // deassert to prevent loading
        //     write = 1'b0; // deassert
        //     vsel = 2'b01; // doesnt matter as write is off
        //     nsel = 2'b00; // doesnt matter as load a/b is off
        //     w = 1'b0;
        
        //     asel = 1'b0; 
        //     bsel = 1'b0;
        //     loadc = 1'b1; // store into c
        //     loads = 1'b1; // store into status
       
        // end


        // Don't need ALUop,shift,readnum,writenum, as these are already conneced from 
        // instruction register to Datapath
        // ALUop determines the operation     



        `WriteReg: begin
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            asel = 1'b0; // deassert
            bsel = 1'b0; // deassert

            nsel = 2'b01; // To choose Rd from Mux31, need nsel = 1
            vsel = 2'b11; // To choose C which is datapath_out
            loadc = 1'b0; // deassert
            loads = 1'b0; // deassert
            write = 1'b1; // enable write

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 

        end

        `HALT: begin
            loada = 1'b0; // deassert
            loadb = 1'b0; // deassert
            asel = 1'b0; // deassert
            bsel = 1'b0; // deassert

            nsel = 2'b01; // dosent matter
            vsel = 2'b11; // dosent matter
            loadc = 1'b0; // deassert
            loads = 1'b0; // deassert
            write = 1'b0; // deassert

            reset_pc = 1'b0; // to select the incremented program counter to store in PC register
            load_pc = 1'b0; // not loading the value into the PC register

            mem_cmd = `MNONE; // Not reading or writing
            addr_sel = 1'b0; // mem_addr is DA_out
            load_ir = 1'b0; // not loading instruction register with instruction from read_data

            // If wrong, might need an extra state for clk
            load_addr = 1'b0; // NO enable loading into Data Address. 

        end



        default: begin 
            nsel = 2'bxx;
            vsel = 2'bxx;
            write = 1'bx;
            loada = 1'bx;
            loadb = 1'bx;
            loadc = 1'bx;
            loads = 1'bx;
            asel = 1'bx;
            bsel = 1'bx;

            reset_pc = 1'bx;
            load_pc = 1'bx;
            mem_cmd = 2'bxx; 
            addr_sel = 1'bx; 
            load_ir = 1'bx;
            load_addr = 1'bx; 

        end
        endcase   
    end
endmodule

// same logic as module register in regfile.v
module registerCPU(data_in,load,clk,R);
parameter n = 1;
input [n-1:0]data_in;
input load;
input clk;
output [n-1:0]R;

wire [n-1:0]next_state_R;

vDFFregCPU #(n) v0(clk, next_state_R, R); //is the flip-flop, the assign statement below is the MUX
assign next_state_R = load ? data_in : R; //(MUX) if load is 1, then data_in , if load is 0, then R0

endmodule

module vDFFregCPU(clk, in, out) ;  // changed name to vDFFreg from vDFF
  parameter n = 1; // width
  input clk ;
  input [n-1:0] in ;
  output [n-1:0] out ;
  reg [n-1:0] out ;

  always @(posedge clk)
    out = in ; //makes the next_state_R the out (present_state)
endmodule

module Mux21CPU(a, b, selector, out);
input [8:0] a, b;
input selector;
output reg [8:0] out;

always @(*) begin 
    case(selector)
    1'b1: out = a; // if selector is 1, out is a
    1'b0: out = b; // if selector is 0, out is b
    endcase
end
endmodule