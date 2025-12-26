`include "./param.v"

module pe(
    input CLK,
    // Row_ID for Weight Buffer
    input [`BIT_ROW_ID-1:0] Row_ID,
    // Input, Weight Input
    input signed [`BIT_DATA-1:0] Data_I_In, 
    input signed [`BIT_DATA-1:0] Data_W_In,
    input EN_W_In,
    input [`BIT_ROW_ID-1:0] EN_ID_In,
    // Psum Input
    input signed [`BIT_PSUM-1:0] Psum_In,
    input [`BIT_ADDR-1:0] Addr_P_In,
    input [`BIT_VALID-1:0] Valid_P_In,
    // Input, Weight output
    output reg [`BIT_DATA-1:0] Data_I_Out,
    output reg signed [`BIT_DATA-1:0] Data_W_Out,
    output reg EN_W_Out,
    output reg [`BIT_ROW_ID-1:0] EN_ID_Out,
    // Psum output
    output reg signed [`BIT_PSUM-1:0] Psum_Out,
    output reg [`BIT_ADDR-1:0]   Addr_P_Out,
    output reg [`BIT_VALID-1:0]  Valid_P_Out
);

reg signed [`BIT_DATA-1:0] Data_W_Buf;

wire signed [`BIT_DATA:0] Data_I_Signed = {1'b0, Data_I_In};

always @(posedge CLK) begin
    Data_I_Out <= Data_I_In;
    Data_W_Out <= Data_W_In;
    EN_W_Out <= EN_W_In;
    EN_ID_Out <= EN_ID_In;
    Addr_P_Out <= Addr_P_In;
    Valid_P_Out <= Valid_P_In;

    if (EN_W_In && (EN_ID_In == Row_ID)) 
        Data_W_Buf <= Data_W_In;

    Psum_Out <= Data_I_In * Data_W_Buf + Psum_In;

    // Psum_Out <= Data_I_Signed * Data_W_Buf + Psum_In;
    // if (Row_ID == 0) begin
    //     $display("  [PE %0d] I=%0d, W=%0d, Psum_In=%0d : Psum_Out=%0d", Row_ID, Data_I_In, Data_W_Buf, Psum_In, Psum_Out);
    //     $display("           Addr_P_In=%0d, Addr_P_Out=%0d, Valid_P_In=%b, Valid_P_Out=%b", Addr_P_In, Addr_P_Out, Valid_P_In, Valid_P_Out);
    // end
    // if (Row_ID == 5) begin
    //     $display("  [PE %0d] I=%0d, W=%0d, Psum_In=%0d : Psum_Out=%0d", Row_ID, Data_I_In, Data_W_Buf, Psum_In, Psum_Out);
    //     $display("           Addr_P_In=%0d, Addr_P_Out=%0d, Valid_P_In=%b, Valid_P_Out=%b", Addr_P_In, Addr_P_Out, Valid_P_In, Valid_P_Out);
    // end
    // if (Row_ID == 6) begin
    //     $display("  [PE %0d] I=%0d, W=%0d, Psum_In=%0d : Psum_Out=%0d", Row_ID, Data_I_In, Data_W_Buf, Psum_In, Psum_Out);
    //     $display("           Addr_P_In=%0d, Addr_P_Out=%0d, Valid_P_In=%b, Valid_P_Out=%b", Addr_P_In, Addr_P_Out, Valid_P_In, Valid_P_Out);
    // end
    // if (Row_ID == 7) begin
    //     $display("  [PE %0d] I=%0d, W=%0d, Psum_In=%0d : Psum_Out=%0d", Row_ID, Data_I_In, Data_W_Buf, Psum_In, Psum_Out);
    //     $display("           Addr_P_In=%0d, Addr_P_Out=%0d, Valid_P_In=%b, Valid_P_Out=%b", Addr_P_In, Addr_P_Out, Valid_P_In, Valid_P_Out);
    // end

    // $display("Psum_Out: %0d", $signed(Psum_Out));
    // if (^Data_W_Buf !== 1'bx) begin
    //     $display("Row_ID: %0d, Data_W_Buf: %0d", Row_ID, Data_W_Buf);
    // end
    // $display("Data_I_In: %d", Data_I_In);
end

endmodule