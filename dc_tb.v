module tb;

 // Inputs
    reg [3:0] opcode;
    wire [8:0] conpipe;
    wire [1:0] aluop;
    reg clk;
    integer i;

    // Instantiate the Unit Under Test (UUT)
    DATA_CONTROL uut (
        .clk(clk)
        .OPcode(opcode),
        .CONTROL_PIPE(conpipe),
        .ALU_OPcode(aluop)
    );

    initial begin
        #1 clk=~clk;
    end

    initial begin
        $monitor( "opcode: %d    conpipe:%b      alu_op:%b      clk=%d" , opcode , conpipe , aluop , clk );
        conpipe=9'h
        
    end

    

endmodule