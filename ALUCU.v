`define ALU_OP_SIZE   2 
`define OP_CODE_SIZE  4
`define ALU_CTRL_SIZE 3
`define INTERNAL_OP_LINE_SIZE 6 //(`ALU_OP_SIZE+`OP_CODE_SIZE)



module ALUCU (
    input [`OP_CODE_SIZE-1:0] OPcode,
    input [`ALU_OP_SIZE-1:0] ALU_OPcode,
    output reg[`ALU_CTRL_SIZE-1:0] ALU_CTRL
);

wire [`INTERNAL_OP_LINE_SIZE-1:0] internal_op_line ;
assign internal_op_line = {ALU_OPcode , OPcode };
always @(internal_op_line) begin
    casex (internal_op_line)
    `INTERNAL_OP_LINE_SIZE'b10xxxx : ALU_CTRL=3'd0;
    `INTERNAL_OP_LINE_SIZE'b01xxxx : ALU_CTRL=3'd1;
    `INTERNAL_OP_LINE_SIZE'b000010 : ALU_CTRL=3'd0;
    `INTERNAL_OP_LINE_SIZE'b000011 : ALU_CTRL=3'd1;
    `INTERNAL_OP_LINE_SIZE'b000100 : ALU_CTRL=3'd2;
    `INTERNAL_OP_LINE_SIZE'b000101 : ALU_CTRL=3'd3;
    `INTERNAL_OP_LINE_SIZE'b000110 : ALU_CTRL=3'd4;
    `INTERNAL_OP_LINE_SIZE'b000111 : ALU_CTRL=3'd5;
    `INTERNAL_OP_LINE_SIZE'b001000 : ALU_CTRL=3'd6;
    `INTERNAL_OP_LINE_SIZE'b001001 : ALU_CTRL=3'd7;
        default: ALU_CTRL=3'd0;
    endcase
end
endmodule