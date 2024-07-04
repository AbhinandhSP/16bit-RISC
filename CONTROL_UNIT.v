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

module CONTROL_UNIT (
    input  [`OP_CODE_SIZE-1:0] OPcode,
    output reg [8:0] CONTROL_PIPE,
    output reg [`ALU_OP_SIZE-1:0] ALU_OP 
);                

    always @(*) begin
      /*  CONTROL_PIPE[8:0]<={
                            bne,
                            write_enable,
                            mem_to_reg,
                            reg_dst,
                            alu_src,
                            dat_mem_write_en,
                            dat_mem_read_en,
                            beq,
                            jump
                        }; */
        case(OPcode) 
            4'b0000:  // LW
            begin
                CONTROL_PIPE[5] = 1'b0;
                CONTROL_PIPE[4] = 1'b1;
                CONTROL_PIPE[6] = 1'b1;
                CONTROL_PIPE[7] = 1'b1;
                CONTROL_PIPE[2] = 1'b1;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b10;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b0001:  // SW
            begin
                CONTROL_PIPE[5] = 1'b0;
                CONTROL_PIPE[4] = 1'b1;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b0;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b1;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b10;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b0010:  // data_processing
            begin
                CONTROL_PIPE[5] = 1'b1;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b1;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b00;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b0011:  // data_processing
            begin
                CONTROL_PIPE[5] = 1'b1;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b1;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b00;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b0100:  // data_processing
            begin
                CONTROL_PIPE[5] = 1'b1;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b1;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b00;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b0101:  // data_processing
            begin
                CONTROL_PIPE[5] = 1'b1;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b1;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b00;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b0110:  // data_processing
            begin
                CONTROL_PIPE[5] = 1'b1;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b1;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b00;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b0111:  // data_processing
            begin
                CONTROL_PIPE[5] = 1'b1;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b1;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b00;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b1000:  // data_processing
            begin
                CONTROL_PIPE[5] = 1'b1;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b1;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b00;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b1001:  // data_processing
            begin
                CONTROL_PIPE[5] = 1'b1;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b1;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b00;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b1011:  // BEQ
            begin
                CONTROL_PIPE[5] = 1'b0;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b0;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b1;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b01;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b1100:  // CONTROL_PIPE[8]
            begin
                CONTROL_PIPE[5] = 1'b0;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b0;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b1;
                ALU_OP = 2'b01;
                CONTROL_PIPE[0] = 1'b0;   
            end
            4'b1101:  // J
            begin
                CONTROL_PIPE[5] = 1'b0;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b0;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b00;
                CONTROL_PIPE[0] = 1'b1;   
            end   
            default: begin
                CONTROL_PIPE[5] = 1'b1;
                CONTROL_PIPE[4] = 1'b0;
                CONTROL_PIPE[6] = 1'b0;
                CONTROL_PIPE[7] = 1'b1;
                CONTROL_PIPE[2] = 1'b0;
                CONTROL_PIPE[3] = 1'b0;
                CONTROL_PIPE[1] = 1'b0;
                CONTROL_PIPE[8] = 1'b0;
                ALU_OP = 2'b00;
                CONTROL_PIPE[0] = 1'b0; 
            end
        endcase
    end
    
endmodule