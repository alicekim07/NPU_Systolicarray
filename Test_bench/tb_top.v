`timescale 1ns / 1ps
`include "param.v"

// Testbench for top module
module tb_top();

    // Testbench variables
    reg CLK, RSTb;
    wire [`BIT_INSTR-1:0] instr_bus;
    wire pulse_bus;
    // wire o_Flag_Finish_Out;
    // wire o_Valid_WB_Out;
    // wire [`BIT_DATAWB-1:0] o_Data_WB_Out;
    // wire o_Flag_Finish = o_Data_WB_Out[`BIT_DATAWB-1];
    // wire o_Valid = o_Data_WB_Out[`BIT_DATAWB-2];
    // wire [`BIT_PSUM-1:0] o_Data = o_Data_WB_Out[`BIT_PSUM-1:0];
    wire o_Flag_Finish;
    wire o_Valid;
    wire [`BIT_PSUM-1:0] o_Data;
    wire instr_stall;
    wire [`BIT_STATE-1:0]o_state_debug;
    // wire [`PE_COL*`BIT_ADDR-1:0] o_sram_weight_addr_out;
    // wire [`PE_COL*`BIT_DATA-1:0] o_sram_weight_din_out;

    // Clock generation
    initial CLK = 1'b0;
    always #20  CLK <= ~CLK;

    // Testbench
    initial begin
        CLK <= 1'b0;
        RSTb <= 1'b0;
        #200;
        repeat(20)  @(negedge CLK);
        RSTb <= 1'b1;

        repeat(3000)  @(negedge CLK);   
        $finish;  
    end

    // Instantiate top module
    top top (
        .CLK(CLK),
        .RSTb(RSTb),
        .i_Instr_In(instr_bus),
        .instr_stall(instr_stall),
        .o_Flag_Finish_Out(o_Flag_Finish),
        .o_Valid_WB_Out(o_Valid),
        .o_Data_WB_Out(o_Data),
        .o_state_debug(o_state_debug),
        .i_instr_pulse(pulse_bus)
        // .o_sram_weight_addr_out(o_sram_weight_addr_out),
        // .o_sram_weight_din_out(o_sram_weight_din_out)
    );

    // Instantiate instruction buffer
    instr_buffer instr_buffer (
        .CLK(CLK),
        .RSTb(RSTb),
        .instr_stall(instr_stall),
        .o_Instr(instr_bus),
        .o_instr_pulse(pulse_bus)
    );

endmodule

// Helper module to buffer instructions
module instr_buffer (
    input CLK,
    input RSTb,
    input instr_stall,
    output reg o_instr_pulse,
    output reg [`BIT_INSTR-1:0] o_Instr
    );

    // Variable declarations
    reg [`BIT_INSTR-1:0] Instr[200000:0];
    reg [17:0]   Count;

    // Provide instruction every 1 cycle if not stalled
    always @(posedge CLK or negedge RSTb) begin
        if (!RSTb) begin
            Count         <= 10'd0;
            o_Instr       <= {`BIT_INSTR{1'b0}};
            o_instr_pulse <= 1'b0;
        end else begin
            o_instr_pulse <= 1'b0;       // 기본 0 (펄스 한 사이클 폭 유지)

            if (!instr_stall) begin
                o_Instr       <= Instr[Count];
                Count         <= Count + 10'd1;
                o_instr_pulse <= 1'b1;   // 이 사이클만 1 → 깔끔한 1클럭 펄스
            end
        end
    end

    reg [31:0] instr_temp;

    integer k;
    integer addr;
    integer bank;

    integer input_end;  // input hex를 읽은 뒤 마지막 index 저장
    integer weight_end; // weight hex를 읽은 뒤 마지막 index 저장
    integer idx;

    parameter INPUT_HEX_SIZE = 784;
    parameter WEIGHT_HEX_SIZE = 784*128;

    initial begin
        // OPVALID(1) / OPCODE(3) / SEL(4)+ADDR(16) or PARAM(20) / DATA(8): total(32)
        // 1) Clear Instr memory
        for (k = 0; k < 200000; k = k + 1) begin
            Instr[k] = 0;
        end

        // 2) PARAM
        Instr[0]  = {`OPVALID, `OPCODE_PARAM, `PARAM_S, 8'd1};
        Instr[1]  = {`OPVALID, `OPCODE_PARAM, `PARAM_IC, 8'd16};
        Instr[2]  = {`OPVALID, `OPCODE_PARAM, `PARAM_IC_WH, 8'd3};     
        Instr[3]  = {`OPVALID, `OPCODE_PARAM, `PARAM_OC, 8'd128};

        // For writeback test (Debug)
        Instr[4]  = {`OPVALID, `OPCODE_WBPARAM, `PARAM_S, 8'd0};
        Instr[5]  = {`OPVALID, `OPCODE_WBPARAM, `PARAM_IC, 8'd0};
        Instr[6]  = {`OPVALID, `OPCODE_WBPARAM, `PARAM_OC, 8'd0};

        // 3) Load input data
        Instr[7] = {`OPVALID, `OPCODE_PARAM, `PARAM_TRG, `TRG_ISRAM}; // Select target SRAM(Input)
        // Instr[8] ~ Instr[8+783]
        $readmemh("instr_input.hex", Instr, 8);

        input_end = 8 + INPUT_HEX_SIZE; // 다음 위치

        // 4) Load weight data
        Instr[input_end] = {`OPVALID, `OPCODE_PARAM, `PARAM_TRG, `TRG_WSRAM};
        $readmemh("instr_mem.hex", Instr, input_end+1);

        weight_end = input_end + WEIGHT_HEX_SIZE; // 다음 위치

        // 5) Execute
        Instr[weight_end]     = {`OPVALID, `OPCODE_PARAM, `PARAM_BASE_WSRAM, 8'd0};
        Instr[weight_end + 1] = {`OPVALID, `OPCODE_EX, 20'd0, 8'd0};
        Instr[weight_end + 2] = {`OPVALID, `OPCODE_NOP, 20'd0, 8'd0};

        // 6) WBPSRAM (출력 받아오기)
        // OPVALID(1) / OPCODE(3) / SEL(4)+ADDR(16) or PARAM(20) / DATA(8): total(32)
        idx = weight_end + 3;

        for (addr = 0; addr < 32; addr = addr + 1) begin   // row == OC // 4
            for (bank = 0; bank < 4; bank = bank + 1) begin
                instr_temp[31]    = `OPVALID;
                instr_temp[30:28] = `OPCODE_WBPSRAM;
                instr_temp[27:24] = bank;     // Bank 0~3
                instr_temp[23:8]  = addr;          // Row index 0~31
                instr_temp[7:0]   = 8'd0;

                Instr[idx] = instr_temp;
                idx = idx + 1;
            end
        end
    end
endmodule
