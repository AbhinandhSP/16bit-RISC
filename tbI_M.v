`define PC_SIZE 16
`define INSTRUCTION_SIZE 16
`define INSTRUCTION_ROM_SIZE 32

module tb;
  reg [15:0]PC;
  wire [15:0]In;
  INSTRUCTION_MEMORY im(.PC(PC),.INSTRUCTION(In));
  integer  i;
  reg [`INSTRUCTION_SIZE-1:0]MEMORY[`INSTRUCTION_ROM_SIZE-1:0];
  reg [15:0]check;  // to check memory and instruction matches or not 
  initial begin
    $readmemb("C:/Users/91989/Desktop/V/c1/MEM.txt", MEMORY,0,`INSTRUCTION_SIZE-1);
    $monitor("PC = %d: In = %b check=%b",PC, In,check);
    PC<=16'd0;
    for (i=0;i<16;i=i+1)begin
      #1
      PC<=PC+2;
      check=MEMORY[i]^In;  // XOR check if both are differnt check bits will become 1 at error point
    end
  end
endmodule

//TESTBENCH FOR INSTRUCTION MEMORY