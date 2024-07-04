module test_Risc_16_bit;

 // Inputs
    reg clk;

    // Instantiate the Unit Under Test (UUT)
    top_module uut (
        .clk(clk)
    );
    initial 
        begin
            clk <=0;
            #500;
            $finish;
        end

    always 
        begin
            #20 clk = ~clk;
        end

endmodule