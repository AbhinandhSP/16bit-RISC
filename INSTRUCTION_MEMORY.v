`define PC_SIZE 16
`define INSTRUCTION_SIZE 16
`define INSTRUCTION_ROM_SIZE 32

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
  $readmemb("C:/Users/91989/Desktop/V/c1/iMEM.txt", MEMORY,0,`INSTRUCTION_SIZE-1);
 end

assign INSTRUCTION=MEMORY[rom_addr];

    
endmodule