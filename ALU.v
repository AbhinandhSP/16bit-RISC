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

