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

/*
INSTRUCTION STRUCTURE: ( 15->0 )
    1. Data Processing Instr:
        [15:12] : 4'OP 
        [11:9]  : 3'Rs1  
        [8:6]   : 3'Rs1 
        [5:3]   : 3'WS 
        [3:0]   : unused 
    2. JUMP
        [15:12] : 4'OP (for JUMP)
        [11:0]  : 12'offset ( address where it is gonna jump)
        NOTE: the current instruction is being executed so the jump addr goes to PC_next, In order to keep
        things similar jump address is left shifted by 2 or jump_addr<<2 goes to PC.
        check below link :
        https://stackoverflow.com/questions/6950230/how-to-calculate-jump-target-address-and-branch-target-address
    3. 
*/
module top_module (
    input clk
);
    wire [8:0] control_pipe;
    wire [`ALU_OP_SIZE-1:0] alu_op_code;
    wire [`OP_CODE_SIZE-1:0] opcode;

    initial
    begin
        $dumpfile("T.vcd");
        $dumpvars(opcode);
    end
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

module DATA_CONTROL (
    input clk,
    input wire [8:0] CONTROL_PIPE,
    input [`ALU_OP_SIZE-1:0] ALU_OPcode,
    output [`OP_CODE_SIZE-1:0] OPcode

);  
    
    //PC
    reg [`PC_SIZE-1:0] pc_current;    // increments only at clk pulse and changes at certain conditions
    wire [`PC_SIZE-1:0] pc_exception; // these to PC are wire type because they must change immediatly
    wire [`PC_SIZE-1:0] pc_next;
    wire [`PC_SIZE-1:0] pc_jump_shift;
    wire [`PC_SIZE-1:0] pc_bne;
    wire [`PC_SIZE-1:0] pc_beq;
    wire [`INSTRUCTION_SIZE-1:0]ext_im;


    //Instr
    wire [`INSTRUCTION_SIZE-1:0] instruction;

    wire [`REG_SIZE-1:0] mem_read_data;

    //GPR ctrl
    wire [`REG_ADDR_BITS-1:0]write_addr; // write section
    wire [`REG_SIZE-1:0] write_data;
    wire [`REG_SIZE-1:0] read_data1; //read section
    wire [`REG_SIZE-1:0] read_data2;
    wire [`REG_ADDR_BITS-1:0] read_addr1; 
    wire [`REG_ADDR_BITS-1:0] read_addr2;

    //ALU
    wire [`REG_SIZE-1:0] second_data;
    wire [`REG_SIZE-1:0]alu_out;
    wire [2:0] alu_control;
    wire flag;

    // CONTROL_PIPE config
    // defn:
    wire jump;
    wire beq;
    wire dat_mem_read_en;
    wire dat_mem_write_en;
    wire alu_src;
    wire reg_dst;
    wire mem_to_reg;
    wire write_enable;
    wire bne;

    //connections
    assign jump = CONTROL_PIPE[0];
    assign beq  = CONTROL_PIPE[1];
    assign dat_mem_read_en= CONTROL_PIPE[2];
    assign dat_mem_write_en= CONTROL_PIPE[3];
    assign alu_src=CONTROL_PIPE[4];
    assign reg_dst=CONTROL_PIPE[5];
    assign mem_to_reg = CONTROL_PIPE[6];
    assign write_enable = CONTROL_PIPE[7];
    assign bne = CONTROL_PIPE[8];

    initial begin
        pc_current <= 16'd0; // starting the program
    end
    


    assign pc_next=pc_current+16'd2; // next instruction is after 16bits=2bytes

    assign ext_im={{(`INSTRUCTION_SIZE-`ALU_SRC_BITS){instruction[`ALU_SRC_BITS-1]}},{instruction[`ALU_SRC_BITS-1:0]}}; //immediate values
    assign pc_jump_shift = {pc_next[15:13],instruction[11:0],1'd0}; // prog_counter_top_4_bits+instruction[11:0]<<2
    assign pc_bne = (bne&(~flag)==1'd1)?(pc_next + {ext_im[14:0],1'b0}):(pc_beq); // when bne and flag=1
    assign pc_beq = (beq&flag==1'd1)?(pc_next + {ext_im[14:0],1'b0}):(pc_next); // when bne=0 , beq =1 otherwise normally pc_next

    assign pc_exception=(jump == 1'b1) ? pc_jump_shift :  pc_bne; // if not jump it check for bne which eventually is pc_next if
    // bne=0 , beq=0 , jump=0

    assign second_data=(alu_src)?(ext_im):(read_data2);
    assign write_addr = (reg_dst==1'd1)?(instruction[5:3]):(instruction[8:6]);
    assign write_data=(mem_to_reg==1'd1)?(mem_read_data):(alu_out);

    assign OPcode = instruction[15:12];

    assign read_addr1=instruction[11:9]; // 15-12 OPcode 11-9 R1 8-6 R2 rest is offset/imm etc.. 
    assign read_addr2=instruction[8:6];


    always @(posedge clk) begin  // program counter increments every clk pulse because each operation in risc happens in one cycle
        pc_current <= pc_exception; // this pc_exception is normally at (pc_current +2 = pc2 ) but at jump,beq,bne etc.. it changes
    end
    
    //MODULE INSTANTIATION

    INSTRUCTION_MEMORY   im(.PC(pc_current),.INSTRUCTION(instruction));

    REG_FILE_MODULE gpr(
        .clk(clk),
        .reg_write_en(write_enable),
        .reg_write_addr(write_addr), // comes 
        .reg_write_data(write_data), // write data either comes from ALU (processed) or from memory (DATA_MEMORY)
        .reg_read_addr1(read_addr1),
        .reg_read_addr2(read_addr2),
        .reg_read_data1(read_data1),
        .reg_read_data2(read_data2)
    );

    ALU alu(
        .A(read_data1),
        .B(second_data),
        .ALU_CTRL(alu_control),
        .ALU_OUT(alu_out),
        .FLAG(flag)
    );

    ALUCU alucu(
        .OPcode(OPcode),
        .ALU_OPcode(ALU_OPcode),
        .ALU_CTRL(alu_control)
    );

    DATA_MEMORY dm(
        .clk(clk),
        .MEMORY_WRITE_ENABLE(dat_mem_write_en),
        .MEMORY_READ_ENABLE(dat_mem_read_en),
        .MEMORY_ACCESS_ADDR(alu_out), 
        // we can change memory access address only from ALU because  RISC processor can not directly access memory 
        // from instruction so only option is ALU 
        .MEMORY_WRITE_DATA(second_data), // second data could be from register 2 or from instruction itself 
        .MEMORY_READ_DATA(mem_read_data) // this is output port so it takes memory to register file
    );

    
endmodule


module DATA_MEMORY (
    input clk,
    input MEMORY_WRITE_ENABLE,
    input MEMORY_READ_ENABLE,
    input [`DATA_MEMORY_SIZE-1:0] MEMORY_ACCESS_ADDR,
    input [`DATA_MEMORY_SIZE-1:0] MEMORY_WRITE_DATA,
    output [`DATA_MEMORY_SIZE-1:0] MEMORY_READ_DATA
);
reg [`DATA_MEMORY_SIZE-1:0]MEMORY[`DATA_MEMORY_STACK_LENGTH-1:0];
assign MEMORY_READ_DATA=(MEMORY_READ_ENABLE==1'b1)?(MEMORY[MEMORY_ACCESS_ADDR]):(16'd0);
integer f;
initial
 begin
    $readmemb("dMEM.txt", MEMORY,0,31);
    $display($time);
 end

always @ (posedge clk) begin
    if (MEMORY_WRITE_ENABLE) MEMORY[MEMORY_ACCESS_ADDR]<=MEMORY_WRITE_DATA;
end
    
endmodule


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


module ALU (
    input [15:0] A,
    input [15:0] B,
    input [2:0] ALU_CTRL,
    output reg [15:0] ALU_OUT,
    output FLAG
);
always @(*) begin

    case(ALU_CTRL)

    3'd0 : ALU_OUT=A+B;
    3'd1 : ALU_OUT=A-B;
    3'd2 : ALU_OUT=~A;
    3'd3 : ALU_OUT=A<<B;
    3'd4 : ALU_OUT=A>>B;
    3'd5 : ALU_OUT=A&B;
    3'd6 : ALU_OUT=A|B;
    3'd7 : ALU_OUT=(A>B)?(16'd1):(16'd0);
    default : ALU_OUT=A+B;

    endcase

end

assign FLAG = (ALU_OUT==16'd0)?(1'd1):(1'd0);
    
endmodule


module INSTRUCTION_MEMORY (
    input [`PC_SIZE-1:0] PC, // program counter
    output [`INSTRUCTION_SIZE-1:0] INSTRUCTION
);
wire [`PC_SIZE-1:1 ]rom_addr=PC[`PC_SIZE-1:1 ];               
// PC<<1 so it will show address 2 bytes after the current instruction because 
// each instruction is 16 bit = 2 bytes and program counter "this" processor holds next instructions address 
// since current one is being executed    
reg [`INSTRUCTION_SIZE-1:0]MEMORY[`INSTRUCTION_ROM_SIZE-1:0];

initial
 begin
  $readmemb("iMEM.txt", MEMORY,0,`INSTRUCTION_SIZE-1);
 end

assign INSTRUCTION=MEMORY[rom_addr];

    
endmodule

`define REG_ADDR_BITS  3 
`define REG_ADDR_SIZE  8

module REG_FILE_MODULE (
    input clk,
    input reg_write_en,
    input [15:0] reg_write_data,
    input [`REG_ADDR_BITS-1:0] reg_write_addr,
    input [`REG_ADDR_BITS-1:0] reg_read_addr1,
    input [`REG_ADDR_BITS-1:0] reg_read_addr2,
    output [15:0] reg_read_data1,
    output [15:0] reg_read_data2

);
 
reg [15:0] reg_array [`REG_ADDR_SIZE-1:0];
integer i;
initial begin
    for (i=0;i<(`REG_ADDR_SIZE);i=i+1) reg_array[i] <= 16'd0; //initializing the array
end

always @(posedge clk) begin
    if(reg_write_en) reg_array[reg_write_addr]<=reg_write_data;
end

assign reg_read_data1=reg_array[reg_read_addr1];
assign reg_read_data2=reg_array[reg_read_addr2];

    
endmodule




