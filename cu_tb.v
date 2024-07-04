module tb;

 // Inputs
    reg [3:0] opcode;
    wire [8:0] conpipe;
    wire [1:0] aluop;
    integer i;

    // Instantiate the Unit Under Test (UUT)
    CONTROL_UNIT uut (
        .OPcode(opcode),
        .CONTROL_PIPE(conpipe),
        .ALU_OP(aluop)
    );

    initial begin
        opcode<=4'd0;
        #1
        $monitor( "opcode: %d    conpipe:%b      alu_op:%b" , opcode , conpipe , aluop );
        for(i=0;i<13;i=i+1) begin
            #1
            opcode<=opcode+1;
        end
        #1
        opcode<=4'd0;
    end

    

endmodule