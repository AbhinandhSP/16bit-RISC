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