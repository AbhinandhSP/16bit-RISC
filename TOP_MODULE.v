`define ALU_OP_SIZE   2 
`define OP_CODE_SIZE  4
`define PC_SIZE 16
`define INSTRUCTION_SIZE 16
`define INSTRUCTION_ROM_SIZE 32
`define REG_ADDR_BITS  3 
`define REG_SIZE 16
`define ALU_SRC_BITS 6 
`define DATA_MEMORY_SIZE 16
`define DATA_MEMORY_STACK_LENGTH 32
`define INTERNAL_OP_LINE_SIZE 6
`define ALU_CTRL_SIZE 3

module top_module (
    input clk
);
    wire [8:0] control_pipe;
    wire [`ALU_OP_SIZE-1:0] alu_op_code;
    wire [`OP_CODE_SIZE-1:0] opcode;


    //module instantiations

    DATA_CONTROL control(
        .clk(clk),
        .CONTROL_PIPE(control_pipe),
        .ALU_OPcode(alu_op_code),
        .OPcode(opcode)
    );

    CONTROL_UNIT cu(
        .OPcode(opcode),
        .CONTROL_PIPE(control_pipe),
        .ALU_OP(alu_op_code)
    );


    
endmodule