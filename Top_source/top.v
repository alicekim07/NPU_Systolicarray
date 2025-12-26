`include "./param.v"

// Module
module top(
    input CLK, RSTb,
    input [`BIT_INSTR-1:0] i_Instr_In,
    input i_instr_pulse,
    output instr_stall,
    output o_Flag_Finish_Out,
    output o_Valid_WB_Out,
    output [`BIT_PSUM-1:0] o_Data_WB_Out,
    output [`BIT_STATE-1:0] o_state_debug
    // output [`PE_COL*`BIT_ADDR-1:0] o_sram_weight_addr_out,
    // output [`PE_COL*`BIT_DATA-1:0] o_sram_weight_din_out
);

// 1.
wire RST;
assign RST = ~RSTb;
// 2. Instruction
wire [`BIT_INSTR-1:0] Instr_In;
// 3. Input,Weight SRAM <-> Controller, Loader
wire [`PE_ROW-1:0] sram_input_we;
wire [`PE_ROW-1:0] sram_input_en;
wire [`PE_ROW*`BIT_ADDR-1:0] sram_input_addr;
wire [`PE_ROW*`BIT_DATA-1:0] sram_input_din;
wire [`PE_ROW*`BIT_DATA-1:0] sram_input_dout;
wire [`PE_COL-1:0] sram_weight_we;
wire [`PE_COL-1:0] sram_weight_en;
wire [`PE_COL*`BIT_ADDR-1:0] sram_weight_addr;
wire [`PE_COL*`BIT_DATA-1:0] sram_weight_din;
wire [`PE_COL*`BIT_DATA-1:0] sram_weight_dout;
// 4. Input Loader <-> Systolic
wire [`PE_ROW*`BIT_DATA-1:0] input_to_systolic;
// 5. Systolic <-> Psum Loader
wire [`PE_COL*`BIT_VALID-1:0] Valid_P_Out;
wire [`PE_COL*`BIT_ADDR-1:0] Addr_P_Out;
wire [`PE_COL*`BIT_PSUM-1:0] Psum_Out;
// 6. Psum Loader <-> Psum SRAM
wire [`PE_COL-1:0] sram_loader_psum_we_a;
wire [`PE_COL-1:0] sram_loader_psum_en_a;
wire [`PE_COL*`BIT_ADDR-1:0] sram_loader_psum_addr_a;
wire [`PE_COL*`BIT_PSUM-1:0] sram_loader_psum_din_a;
// 7. Psum SRAM <-> Systolic Input
wire [`PE_COL-1:0] sram_psum_we_a;
wire [`PE_COL-1:0] sram_psum_en_a;
wire [`PE_COL*`BIT_ADDR-1:0] sram_psum_addr_a;
wire [`PE_COL*`BIT_PSUM-1:0] sram_psum_din_a;
wire [`PE_COL-1:0] sram_psum_we_b;
wire [`PE_COL-1:0] sram_psum_en_b;
wire [`PE_COL*`BIT_ADDR-1:0] sram_psum_addr_b;
wire [`PE_COL*`BIT_PSUM-1:0] sram_psum_dout_b;
// 8. Controller <-> Systolic
wire [`BIT_ROW_ID-1:0] systolic_en_row_id;
wire [`PE_COL-1:0] systolic_en_w;
wire [`PE_COL*`BIT_ADDR-1:0] systolic_addr_p;
wire [`PE_COL*`BIT_VALID-1:0] systolic_valid_p;
// 9. Ctrl <-> Psum SRAM
wire [`PE_COL-1:0] sram_ctrl_psum_we_a;
wire [`PE_COL-1:0] sram_ctrl_psum_en_a;
wire [`PE_COL*`BIT_ADDR-1:0] sram_ctrl_psum_addr_a;
wire [`PE_COL*`BIT_PSUM-1:0] sram_ctrl_psum_din_a;
// 10. Ctrl, Loader, muxing Psum Control
wire psum_sel_ctrl;
assign sram_psum_addr_a = psum_sel_ctrl ? sram_ctrl_psum_addr_a : sram_loader_psum_addr_a;
assign sram_psum_din_a = psum_sel_ctrl ? sram_ctrl_psum_din_a : sram_loader_psum_din_a;
assign sram_psum_we_a = psum_sel_ctrl ? sram_ctrl_psum_we_a : sram_loader_psum_we_a;
assign sram_psum_en_a = psum_sel_ctrl ? sram_ctrl_psum_en_a : sram_loader_psum_en_a;

// 11. Unused
// wire [`PE_COL*`BIT_PSUM-1:0] sram_psum_dout_a;
// wire [`PE_COL*`BIT_PSUM-1:0] sram_psum_din_b;

// 12. Weight Muxing
wire weight_to_systolic_sel;
wire [`PE_COL*`BIT_DATA-1:0] weight_to_systolic;
assign weight_to_systolic = weight_to_systolic_sel ? sram_weight_dout : 0;

// 13. Instruction Pulse(Wrapper)
wire instr_pulse;

dffq #(`BIT_INSTR) dffq_instr (
    .CLK(CLK),
    .D(i_Instr_In),
    .Q(Instr_In)
);

// dffq #(1) dffq_pulse_1 (
//   .CLK(CLK),
//   .D(i_instr_pulse),
//   .Q(instr_pulse)
// );

systolic_ctrl systolic_ctrl(
    .CLK(CLK),
    .RSTb(RSTb),
    .Instr_In(Instr_In),
    .Flag_Finish_Out(o_Flag_Finish_Out),
    .Valid_WB_Out(o_Valid_WB_Out),
    .Data_WB_Out(o_Data_WB_Out),
    // For SRAM Input Writing
    .sram_input_we(sram_input_we), // SRAM Write Enable
    .sram_input_en(sram_input_en), // SRAM Enable
    .sram_input_addr(sram_input_addr), // 26112 Data, More than 15 Bit needed
    .sram_input_din(sram_input_din), // Data to write
    // For SRAM Weight Writing
    .sram_weight_we(sram_weight_we), // SRAM Write Enable
    .sram_weight_en(sram_weight_en), // SRAM Enable
    .sram_weight_addr(sram_weight_addr), // 26112 Data, More than 15 Bit needed
    .sram_weight_din(sram_weight_din), // Data to write
    // For SRAM Psum Writing (A Port)
    .sram_psum_we_a(sram_ctrl_psum_we_a), // SRAM Write Enable
    .sram_psum_en_a(sram_ctrl_psum_en_a), // SRAM Enable
    .sram_psum_addr_a(sram_ctrl_psum_addr_a), // Address to write
    .sram_psum_din_a(sram_ctrl_psum_din_a), // Data to write
    // For SRAM Psum Reading (B Port)
    .sram_psum_we_b(sram_psum_we_b), // SRAM Write Enable
    .sram_psum_en_b(sram_psum_en_b), // SRAM Enable
    .sram_psum_addr_b(sram_psum_addr_b), // Address to write
    .sram_psum_dout_b(sram_psum_dout_b), // Data to write
    // For Systolic Weight Control
    .systolic_en_row_id(systolic_en_row_id), // Row ID for systolic array
    .systolic_en_w(systolic_en_w), // Enable for systolic array
    // For Systolic Psum In Control
    .systolic_addr_p(systolic_addr_p), // Address for systolic array
    .systolic_valid_p(systolic_valid_p), // Valid for systolic array
    // For stall
    .instr_stall(instr_stall),
    // For Psum SRAM get control
    .psum_sel_ctrl(psum_sel_ctrl), // Select control for Psum SRAM
    // For Weight muxing
    .weight_to_systolic_sel(weight_to_systolic_sel), // Select control for weight
    // For debug
    .state_out(o_state_debug), // State output
    // .sram_weight_addr_out(o_sram_weight_addr_out),
    // .sram_weight_din_out(o_sram_weight_din_out),
    // For instruction stablize
    .instr_pulse(i_instr_pulse)
    // To avoid collision
    // .sram_loader_psum_en_a(sram_loader_psum_en_a)
);


systolic systolic(
    .CLK(CLK),
    // Control Weight, Input
    .i_Data_I_In(input_to_systolic),
    .i_Data_W_In(weight_to_systolic),
    .i_EN_W_In(systolic_en_w),
    .i_EN_ID_In(systolic_en_row_id),
    // Psum Data
    .i_Psum_In(sram_psum_dout_b),
    .o_Psum_Out(Psum_Out),
    // Control Psum Data
    .i_Addr_P_In(systolic_addr_p),
    .i_Valid_P_In(systolic_valid_p),
    .o_Addr_P_Out(Addr_P_Out),
    .o_Valid_P_Out(Valid_P_Out)
);
        integer k;
        // always @(negedge CLK) begin
        //     for (k = 0; k < `PE_COL; k = k + 1) begin
        //         if (systolic_valid_p[k*`BIT_VALID +: `BIT_VALID]) begin
        //             $display("  Systolic Input");
        //             $display("      Bank: %0d, Psum: %0d, Addr: %0d, Valid: %0d",
        //                 k,
        //                 $signed(sram_psum_dout_b[k*`BIT_PSUM +: `BIT_PSUM]),
        //                 systolic_addr_p[k*`BIT_ADDR +: `BIT_ADDR],
        //                 systolic_valid_p[k*`BIT_VALID +: `BIT_VALID]
        //             );
        //         end
        //     end
        // end
        always @(posedge CLK) begin
            for (k = 0; k < `PE_COL; k = k + 1) begin
                if (Valid_P_Out[k*`BIT_VALID +: `BIT_VALID]) begin
                    $display("      State: %0x, Bank: %0d, Psum: %0d, Addr: %0d, Valid: %0d",
                        o_state_debug,
                        k,
                        $signed(Psum_Out[k*`BIT_PSUM +: `BIT_PSUM]),
                        Addr_P_Out[k*`BIT_ADDR +: `BIT_ADDR],
                        Valid_P_Out[k*`BIT_VALID +: `BIT_VALID]
                    );
                end
            end
        end

genvar i;
generate for (i=0;i<`PE_ROW;i=i+1) begin: Loop_I
    blk_mem_gen_0_sp   sram_i (
        .clka(CLK),
        .addra(sram_input_addr[i*`BIT_ADDR+:`BIT_ADDR]),
        .dina(sram_input_din[i*`BIT_DATA+:`BIT_DATA]),
        .douta(sram_input_dout[i*`BIT_DATA+:`BIT_DATA]),
        .ena(sram_input_en[i]),
        .wea(sram_input_we[i])
    );
end
endgenerate

generate for (i=0;i<`PE_COL;i=i+1) begin: Loop_W
    blk_mem_gen_0_sp   sram_w (
        .clka(CLK),
        .addra(sram_weight_addr[i*`BIT_ADDR+:`BIT_ADDR]),
        .dina(sram_weight_din[i*`BIT_DATA+:`BIT_DATA]),
        .douta(sram_weight_dout[i*`BIT_DATA+:`BIT_DATA]),
        .ena(sram_weight_en[i]),
        .wea(sram_weight_we[i])
    );
    blk_mem_gen_1_dp sram_psum (
        // A Port (Writing)
        .clka(CLK),
        .addra(sram_psum_addr_a[i*`BIT_ADDR+:`BIT_ADDR]),
        .dina(sram_psum_din_a[i*`BIT_PSUM+:`BIT_PSUM]),
        .douta(), // unused
        .ena(sram_psum_en_a[i]),
        .wea(sram_psum_we_a[i]),
        // B Port (Reading)
        .clkb(CLK),
        .addrb(sram_psum_addr_b[i*`BIT_ADDR+:`BIT_ADDR]),
        .dinb({`BIT_PSUM{1'b0}}),
        .doutb(sram_psum_dout_b[i*`BIT_PSUM+:`BIT_PSUM]),
        .enb(sram_psum_en_b[i]),
        .web(sram_psum_we_b[i])
    );
end
endgenerate


        // always @(negedge CLK) begin
        //     for (k = 0; k < `PE_COL; k = k + 1) begin
        //         if (sram_psum_en_a[k]) begin
        //             $display("      State:%0x, Bank:%0d, Addr_a:%0d, en_a:%0d, en_b:%0d, Addr_b:%0d",
        //                 o_state_debug,
        //                 k,
        //                 sram_psum_addr_a[k*`BIT_ADDR+:`BIT_ADDR],
        //                 sram_psum_en_a[k],
        //                 sram_psum_en_b[k],
        //                 sram_psum_addr_b[k*`BIT_ADDR+:`BIT_ADDR]
        //             );
        //         end
        //     end
        // end


        // always @(negedge CLK) begin
        //     $display(" PSUM A Port IN Write");
        //     for (k = 0; k < `PE_COL; k = k + 1) begin
        //         $display("      Bank:%0d, Addr_a:%0d, din_a:%0d, en_a:%0d, we_a:%0d",
        //             k,
        //             sram_psum_addr_a[k*`BIT_ADDR+:`BIT_ADDR],
        //             $signed(sram_psum_din_a[k*`BIT_PSUM +:`BIT_PSUM]),
        //             sram_psum_en_a[k],
        //             sram_psum_we_a[k]
        //         );
        //     end
        // end

        // always @(negedge CLK) begin
        //     $display("  PSUM B Port Out Read");
        //     for (k = 0; k < `PE_COL; k = k + 1) begin
        //         $display("      Bank:%0d, Addr_b:%0d, dout_b:%0d, en_b:%0d, we_b:%0d",
        //             k,
        //             sram_psum_addr_b[k*`BIT_ADDR+:`BIT_ADDR],
        //             $signed(sram_psum_dout_b[k*`BIT_PSUM +:`BIT_PSUM]),
        //             sram_psum_en_b[k],
        //             sram_psum_we_b[k]
        //         );
        //     end
        // end

        // Debugging
        integer j;
        // // Weight SRAM Check
        // always @(posedge CLK) begin
        //     for (j = 0; j < `PE_COL; j = j + 1) begin
        //         if (sram_weight_en[j] && sram_weight_we[j]) begin
        //             $display("[Weight Write] Bank : %0d, Addr : %0d, Data : %0d", j, sram_weight_addr[j*`BIT_ADDR+:`BIT_ADDR], sram_weight_din[j*`BIT_DATA+:`BIT_DATA]);
        //         end
        //     end
        // end
        // // Input SRAM Check
        // always @(posedge CLK) begin
        //     for (j = 0; j < `PE_ROW; j = j + 1) begin
        //         if (sram_input_en[j] && sram_input_we[j]) begin
        //             $display("[Input Write] Bank : %0d, Addr : %0d, Data : %0d", j, sram_input_addr[j*`BIT_ADDR+:`BIT_ADDR], sram_input_din[j*`BIT_DATA+:`BIT_DATA]);
        //         end
        //     end
        // end
        // // Psum SRAM Check
        // always @(posedge CLK) begin
        //     for (j = 0; j < `PE_COL; j = j + 1) begin
        //         if (sram_psum_en_a[j] && sram_psum_we_a[j] && j == 0) begin
        //             $display("[Psum Write] Bank : %0d, Addr : %0d, Data : %0d", j, sram_psum_addr_a[j*`BIT_ADDR+:`BIT_ADDR], sram_psum_din_a[j*`BIT_PSUM+:`BIT_PSUM]);
        //         end
        //     end
        // end


systolic_loader_i systolic_loader_i (
    .CLK(CLK),
    .i_Data_I_In(sram_input_dout),
    .o_Data_I_In(input_to_systolic)
);

systolic_loader_p systolic_loader_p (
    .CLK(CLK),
    .RSTb(RSTb),
    // Get Psum data from systolic
    .Valid_P_Out(Valid_P_Out),
    .Addr_P_Out(Addr_P_Out),
    .Psum_Out(Psum_Out),
    // For SRAM(Psum) Writing (A Port)
    .sram_psum_we_a(sram_loader_psum_we_a), // SRAM Write Enable
    .sram_psum_en_a(sram_loader_psum_en_a), // SRAM Enable
    .sram_psum_addr_a(sram_loader_psum_addr_a), // Address to write
    .sram_psum_din_a(sram_loader_psum_din_a) // Data to write
);



endmodule
