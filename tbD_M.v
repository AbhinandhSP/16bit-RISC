`define DATA_MEMORY_SIZE 16
`define DATA_MEMORY_STACK_LENGTH 32
module tb;

reg clk;

initial begin 
    clk = 0;
    forever begin
        #1 clk = ~clk;
    end 
end

reg wenable ;
wire renable;
reg  [15:0] addr;
reg  [15:0] wdata;
wire  [15:0] rdata;

DATA_MEMORY dm(
    .clk(clk),
    .MEMORY_WRITE_ENABLE(wenable),
    .MEMORY_READ_ENABLE(renable),
    .MEMORY_ACCESS_ADDR(addr),
    .MEMORY_WRITE_DATA(wdata),
    .MEMORY_READ_DATA(rdata) 
);

integer  i;
initial begin
    $dumpfile("test.vcd");
    $dumpvars(wenable,renable,addr,wdata,rdata,clk);
end
initial begin
    $monitor("ADDR=%d : RM= %d , WM=%d",addr, rdata , wdata);
    addr<=16'd0;
    wdata<=16'd0;
    for (i=0;i<64;i=i+1) begin
        if(i%2==0) #5 addr<=addr+2;
        wenable<=1;
        #5
        wdata<=wdata+1;
        #5
        wenable<=0;
    end
end

endmodule

//TESTBENCH FOR INSTRUCTION MEMORY