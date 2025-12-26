`include "./param.v"


module systolic_ctrl(
    // # region Systolic Array Control Module
        input CLK, RSTb,
        input [`BIT_INSTR-1:0] Instr_In,
        input instr_pulse,
        // For WriteBack(Read Path)
        output Flag_Finish_Out,
        output Valid_WB_Out,
        output [`BIT_PSUM-1:0] Data_WB_Out,
        // For SRAM Input Writing
        output [`PE_ROW-1:0] sram_input_we, // SRAM Write Enable
        output [`PE_ROW-1:0] sram_input_en , // SRAM Enable
        output [`PE_ROW*`BIT_ADDR-1:0] sram_input_addr, // 26112 Data, More than 15 Bit needed
        output [`PE_ROW*`BIT_DATA-1:0] sram_input_din, // Data to write
        // For SRAM Weight Writing
        output [`PE_COL-1:0] sram_weight_we , // SRAM Write Enable
        output [`PE_COL-1:0] sram_weight_en , // SRAM Enable
        output [`PE_COL*`BIT_ADDR-1:0] sram_weight_addr, // 26112 Data, More than 15 Bit needed
        output [`PE_COL*`BIT_DATA-1:0] sram_weight_din, // Data to write
        // For SRAM Psum Writing (A Port)
        output [`PE_COL-1:0] sram_psum_we_a, // SRAM Write Enable
        output [`PE_COL-1:0] sram_psum_en_a, // SRAM Enable
        output [`PE_COL*`BIT_ADDR-1:0] sram_psum_addr_a, // Address to write
        output [`PE_COL*`BIT_PSUM-1:0] sram_psum_din_a, // Data to write
        // For SRAM Psum Reading (B Port)
        output [`PE_COL-1:0] sram_psum_we_b, // SRAM Write Enable
        output [`PE_COL-1:0] sram_psum_en_b, // SRAM Enable
        output [`PE_COL*`BIT_ADDR-1:0] sram_psum_addr_b, // Address to read
        input [`PE_COL*`BIT_PSUM-1:0] sram_psum_dout_b, // Data to read
        // For Systolic Weight Control
        output [`BIT_ROW_ID-1:0] systolic_en_row_id, // Row ID for systolic array
        output [`PE_COL-1:0] systolic_en_w, // Enable for systolic array
        // For Systolic Psum In Control
        output [`PE_COL*`BIT_ADDR-1:0] systolic_addr_p, // Address for systolic array
        output [`PE_COL*`BIT_VALID-1:0] systolic_valid_p, // Valid for systolic array
        // For Stall
        output instr_stall,
        // For Psum SRAM get control
        output psum_sel_ctrl,
        // For Weight muxing
        output weight_to_systolic_sel,
        // For debug
        output [`BIT_STATE-1:0] state_out // Debug output for state
        // output [`PE_COL*`BIT_ADDR-1:0] sram_weight_addr_out,
        // output [`PE_COL*`BIT_DATA-1:0] sram_weight_din_out
        // To avoid Collision
        // output [`PE_COL-1:0] sram_loader_psum_en_a
    // #endregion
    );

    // o_Flag_Finish : Finish Flag ('1' When OPCODE_EX has been finished. '0' otherwise), o_Flag Finish returns to '0' if writeback starts
    // o_Valid : Valid Output Data Flag('1' when o_Data is valid. '0' otherwise)
    // o_Data : 24-bit Output Data

    // # 0) Parameter
    // # region parameter
        parameter integer WB_READY_WAIT_CYCLES = 2;
        parameter integer WB_VALID_HOLD_CYCLES = 3;
        localparam integer WB_READY_CNT_WIDTH = 2;
        localparam integer WB_CNT_WIDTH = 2;
        parameter integer BYTES_PER_ELEM = 1; // INT8
        parameter integer ELEMS_PER_WORD = 8; // 8개의 INT8 = 1 Word
        parameter integer BYTES_PER_WORD = BYTES_PER_ELEM * ELEMS_PER_WORD;
        parameter integer ADDR_SHIFT = 3;
        localparam integer LANE_ISRAM = 8;
        localparam integer LANE_WSRAM = 4;
        localparam integer LANE_PSRAM = 4;
        localparam integer SELW_ISRAM = 3; // clog2(8)
        localparam integer SELW_WSRAM = 2; // clog2(4)
        localparam integer SELW_PSRAM = 2; // clog2(4)
    // # endregion

    // 1) 외부 상태/카운터 선언부 : q/d 쌍 만들기
    // # region External State Declaration
        // For WriteBack(Read Path)
        reg Flag_Finish_Out_q, Flag_Finish_Out_d;
        reg Valid_WB_Out_q, Valid_WB_Out_d;
        reg [`BIT_PSUM-1:0] Data_WB_Out_q, Data_WB_Out_d;
        // For SRAM Input Writing
        reg [`PE_ROW-1:0] sram_input_we_q, sram_input_we_d; // SRAM Write Enable
        reg [`PE_ROW-1:0] sram_input_en_q, sram_input_en_d; // SRAM Enable
        reg [`PE_ROW*`BIT_ADDR-1:0] sram_input_addr_q, sram_input_addr_d; // 26112 Data, More than 15 Bit needed
        reg [`PE_ROW*`BIT_DATA-1:0] sram_input_din_q, sram_input_din_d; // Data to write
        // For SRAM Weight Writing
        reg [`PE_COL-1:0] sram_weight_we_q, sram_weight_we_d; // SRAM Write Enable
        reg [`PE_COL-1:0] sram_weight_en_q, sram_weight_en_d; // SRAM Enable
        reg [`PE_COL*`BIT_ADDR-1:0] sram_weight_addr_q, sram_weight_addr_d; // 26112 Data, More than 15 Bit needed
        reg [`PE_COL*`BIT_DATA-1:0] sram_weight_din_q, sram_weight_din_d; // Data to write
        // For SRAM Psum Writing (A Port)
        reg [`PE_COL-1:0] sram_psum_we_a_q, sram_psum_we_a_d; // SRAM Write Enable
        reg [`PE_COL-1:0] sram_psum_en_a_q, sram_psum_en_a_d; // SRAM Enable
        reg [`PE_COL*`BIT_ADDR-1:0] sram_psum_addr_a_q, sram_psum_addr_a_d; // Address to write
        reg [`PE_COL*`BIT_PSUM-1:0] sram_psum_din_a_q, sram_psum_din_a_d; // Data to write
        // For SRAM Psum Reading (B Port)
        reg [`PE_COL-1:0] sram_psum_we_b_q, sram_psum_we_b_d; // SRAM Write Enable
        reg [`PE_COL-1:0] sram_psum_en_b_q, sram_psum_en_b_d; // SRAM Enable
        reg [`PE_COL*`BIT_ADDR-1:0] sram_psum_addr_b_q, sram_psum_addr_b_d; // Address to read
        // For Systolic Weight Control
        reg [`BIT_ROW_ID-1:0] systolic_en_row_id_q, systolic_en_row_id_d; // Row ID for systolic array
        reg [`PE_COL-1:0] systolic_en_w_q, systolic_en_w_d; // Enable for systolic array
        // For Systolic Psum In Control
        reg [`PE_COL*`BIT_ADDR-1:0] systolic_addr_p_q, systolic_addr_p_d; // Address for systolic array
        reg [`PE_COL*`BIT_VALID-1:0] systolic_valid_p_q, systolic_valid_p_d; // Valid for systolic array
        // For Psum SRAM get control
        reg psum_sel_ctrl_q, psum_sel_ctrl_d;
    // # endregion

    // 2) 내부 상태/카운터 선언부 : q/d 쌍 만들기
    // # region Internal State Declaration
        // State Variables
        reg [`BIT_STATE:0] state_q, state_d;
        reg [`BIT_STATE:0] previous_state_q, previous_state_d;
        reg [`BIT_STATE:0] next_task_state_q, next_task_state_d;
        // Define the opcode, param, and data
        reg opvalid_q, opvalid_d;
        reg [`BIT_OPCODE-1:0] opcode_q, opcode_d;
        reg [`BIT_PARAM-1:0] param_q, param_d;
        reg [`BIT_SEL-1:0] sel_q, sel_d;
        reg [`BIT_ADDR-1:0] addr_q, addr_d;
        reg [`BIT_DATA-1:0] data_q, data_d;
        // Instruction register
        reg [`BIT_INSTR-1:0] instr_reg_q, instr_reg_d;
        // Define Parameters
        reg [`BIT_PARAM-1:0] Param_S_q, Param_S_d;
        reg [`BIT_PARAM-1:0] Param_OC_q, Param_OC_d;
        reg [`BIT_PARAM-1:0] Param_IC_q, Param_IC_d;
        reg [`BIT_PARAM-1:0] Param_TRG_q, Param_TRG_d;
        // For PSUM INIT
        reg [`BIT_ADDR-1:0] psum_init_addr_q, psum_init_addr_d; // Psum init count
        // IC, OC for Execute
        reg [`BIT_PARAM:0] State_IC_q, State_IC_d;
        reg [`BIT_PARAM:0] State_OC_q, State_OC_d;
        reg [`BIT_PARAM:0] Next_IC_q, Next_IC_d;
        reg [`BIT_PARAM:0] Next_OC_q, Next_OC_d;
        reg [`BIT_PARAM:0] Run_IC_q, Run_IC_d; // # of Enabled IC (PE Rows)
        reg [`BIT_PARAM:0] Run_OC_q, Run_OC_d; // # of Enabled OC (PE Cols)
        // Count
        reg [3:0] load_cnt_q, load_cnt_d; // Load count for systolic array
        reg [7:0] run_cnt_q, run_cnt_d; // Run count for systolic array
        reg [31:0] clk_cnt_q, clk_cnt_d; // Clock count for systolic array
        //
        reg [`PE_COL*`BIT_VALID-1:0] systolic_valid_p_raw_q, systolic_valid_p_raw_d; // Valid for systolic array
        //
        reg [WB_CNT_WIDTH-1:0] wb_valid_cnt_q, wb_valid_cnt_d; // Counter for WB valid hold
        reg [WB_READY_CNT_WIDTH-1:0] wb_ready_cnt_q, wb_ready_cnt_d; // Counter for WB ready wait
        //
        reg [`BIT_DATA-1:0] Param_BASE_WSRAM_q, Param_BASE_WSRAM_d;
        reg [`BIT_DATA-1:0] Param_BASE_WSRAM_WH_q, Param_BASE_WSRAM_WH_d;

    // # endregion

    // 3) 출력 포트 연결은 항상 _q 쪽으로
    // # region Output Port Assignment
        assign Flag_Finish_Out = Flag_Finish_Out_q;
        assign Valid_WB_Out = Valid_WB_Out_q;
        assign Data_WB_Out = Data_WB_Out_q;
        assign sram_input_we = sram_input_we_q;
        assign sram_input_en = sram_input_en_q;
        assign sram_input_addr = sram_input_addr_q;
        assign sram_input_din = sram_input_din_q;
        assign sram_weight_we = sram_weight_we_q;
        assign sram_weight_en = sram_weight_en_q;
        assign sram_weight_addr = sram_weight_addr_q;
        assign sram_weight_din = sram_weight_din_q;
        assign sram_psum_we_a = sram_psum_we_a_q;
        assign sram_psum_en_a = sram_psum_en_a_q;
        assign sram_psum_addr_a = sram_psum_addr_a_q;
        assign sram_psum_din_a = sram_psum_din_a_q;
        assign sram_psum_we_b = sram_psum_we_b_q;
        assign sram_psum_en_b = sram_psum_en_b_q; // & ~sram_loader_psum_en_a & ~sram_psum_en_a_q; // Disable if loader or A port is writing
        assign sram_psum_addr_b = sram_psum_addr_b_q;
        assign systolic_en_row_id = systolic_en_row_id_q;
        assign systolic_en_w = systolic_en_w_q;
        assign systolic_addr_p = systolic_addr_p_q;
        assign systolic_valid_p = systolic_valid_p_q;
        assign psum_sel_ctrl = psum_sel_ctrl_q;
    // # endregion

    // 4) 내부 wire 신호
    // # region Internal Wire Declaration
        wire [`BIT_PARAM:0] Base_I_calc = Param_S_q * (State_IC_q / `PE_ROW);
        wire [`BIT_PARAM:0] Base_W_calc = State_IC_q + Param_IC_q * (State_OC_q / `PE_COL);
        // instr_reg_q에서만 필드 추출 (Instr_In 직접 참조 금지!)
        wire opvalid_w = instr_reg_q[(`BIT_DATA + `BIT_PARAM + `BIT_OPCODE) +: `BIT_VALID];
        wire [`BIT_OPCODE-1:0] opcode_w = instr_reg_q[(`BIT_DATA + `BIT_PARAM) +: `BIT_OPCODE];
        wire [`BIT_PARAM-1:0]  param_w  = instr_reg_q[`BIT_DATA +: `BIT_PARAM];
        wire [`BIT_SEL-1:0]    sel_w    = instr_reg_q[`BIT_DATA + `BIT_ADDR +: `BIT_SEL];
        wire [`BIT_ADDR-1:0]   addr_w   = instr_reg_q[`BIT_DATA +: `BIT_ADDR];
        wire [`BIT_DATA-1:0]   data_w   = instr_reg_q[0 +: `BIT_DATA];
        wire [`BIT_ADDR-1:0]   Base_W_full;
 
        wire [SELW_ISRAM-1:0] isram_sel = sel_q[SELW_ISRAM-1:0];
        wire [SELW_WSRAM-1:0] wsram_sel = sel_q[SELW_WSRAM-1:0];
        wire [SELW_PSRAM-1:0] psram_sel = sel_q[SELW_PSRAM-1:0];
    // # endregion
    // 4) 파생 신호(콤비)
    // # region Derived Signals
        assign instr_stall = (state_q != `FETCH);
        assign weight_to_systolic_sel = (load_cnt_q < Run_IC_q+1) ? 1'b1 : 1'b0; // Weight to systolic array
        assign Base_W_full = {Param_BASE_WSRAM_WH_q[`BIT_DATA-1:0], Param_BASE_WSRAM_q[`BIT_DATA-1:0]};
    // # endregion

    // 5) 임시 Combination 신호
    // # region Temporay Combinational Signals
        integer i;
        integer width;
        integer pos;
        integer total_len;
        integer win_start;
        integer win_end;
        reg [`PE_COL-1:0] oc_mask;
        reg [`BIT_ADDR-1:0] pattern_array [0:`PE_ROW-1]; // Pattern array for systolic array
        reg [`BIT_ADDR-1:0] pattern_array_d [0:`PE_ROW-1]; // Pattern array for systolic array
        reg [`PE_COL-1:0] valid_window;
    // # endregion

    // 6) Combination Block : 기본 hold 후 state 별로 *_d 갱신
    // # region Combinational Block
        always @(*) begin
            // 0. Default hold
            // # region Default Hold
                Flag_Finish_Out_d = Flag_Finish_Out_q;
                Valid_WB_Out_d = Valid_WB_Out_q;
                Data_WB_Out_d = Data_WB_Out_q;
                sram_input_we_d = sram_input_we_q;
                sram_input_en_d = sram_input_en_q;
                sram_input_addr_d = sram_input_addr_q;
                sram_input_din_d = sram_input_din_q;
                sram_weight_we_d = sram_weight_we_q;
                sram_weight_en_d = sram_weight_en_q;
                sram_weight_addr_d = sram_weight_addr_q;
                sram_weight_din_d = sram_weight_din_q;
                sram_psum_we_a_d = sram_psum_we_a_q;
                sram_psum_en_a_d = sram_psum_en_a_q;
                sram_psum_addr_a_d = sram_psum_addr_a_q;
                sram_psum_din_a_d = sram_psum_din_a_q;
                sram_psum_we_b_d = sram_psum_we_b_q;
                sram_psum_en_b_d = sram_psum_en_b_q;
                sram_psum_addr_b_d = sram_psum_addr_b_q;
                systolic_en_row_id_d = systolic_en_row_id_q;
                systolic_en_w_d = systolic_en_w_q;
                systolic_addr_p_d = systolic_addr_p_q;
                systolic_valid_p_d = systolic_valid_p_q;
                psum_sel_ctrl_d = psum_sel_ctrl_q;
                state_d = state_q;
                previous_state_d = previous_state_q;
                next_task_state_d = next_task_state_q;
                clk_cnt_d = clk_cnt_q + 1;
                opvalid_d = opvalid_q;
                opcode_d = opcode_q;
                param_d = param_q;
                sel_d = sel_q;
                addr_d = addr_q;
                data_d = data_q;
                Param_S_d = Param_S_q;
                Param_OC_d = Param_OC_q;
                Param_IC_d = Param_IC_q;
                Param_TRG_d = Param_TRG_q;

                psum_init_addr_d = psum_init_addr_q;

                State_IC_d = State_IC_q;
                State_OC_d = State_OC_q;
                Next_IC_d = Next_IC_q;
                Next_OC_d = Next_OC_q;
                Run_IC_d = Run_IC_q;
                Run_OC_d = Run_OC_q;
                load_cnt_d = load_cnt_q;
                run_cnt_d = run_cnt_q;
                systolic_valid_p_raw_d = systolic_valid_p_raw_q;
                wb_valid_cnt_d = wb_valid_cnt_q;
                wb_ready_cnt_d = wb_ready_cnt_q;

                Param_BASE_WSRAM_d = Param_BASE_WSRAM_q;
                Param_BASE_WSRAM_WH_d = Param_BASE_WSRAM_WH_q;
                //
                instr_reg_d = instr_reg_q;
            // # endregion
            // # region Default derived signals
                oc_mask = {`PE_COL{1'b0}};
                valid_window = {`PE_COL{1'b0}};
            // # endregion

            // 1. Next State Logic
            case (state_q)
                `IDLE: begin
                    previous_state_d = `IDLE;
                    state_d = `FETCH;
                end
                `FETCH: begin
                    previous_state_d = `FETCH;
                    state_d = `STABLIZE;
                end
                `STABLIZE: begin
                    if (instr_pulse) begin
                        previous_state_d = `STABLIZE;
                        state_d = `DECODE;
                    end else begin
                        previous_state_d = `STABLIZE;
                        state_d = `DECODE;
                        //state_d = `FETCH;
                    end
                end
                `DECODE: begin
                    previous_state_d = `DECODE;
                    state_d = `EXECUTE; 
                end
                `EXECUTE: begin
                    previous_state_d = `EXECUTE;
                    if (opvalid_q) begin
                        case (opcode_q)
                            `OPCODE_NOP:       state_d = `FETCH;
                            `OPCODE_PARAM:     state_d = `PARAM_SET;
                            `OPCODE_LDSRAM: begin
                                if (Param_TRG_q == `TRG_PSRAM) begin
                                    state_d = `SET_CTRL;
                                    next_task_state_d = `WRITE_SRAM;
                                end else begin
                                    state_d = `WRITE_SRAM;
                                end
                            end
                            `OPCODE_STSRAM:    state_d = `READ_SRAM;
                            `OPCODE_EX: begin
                                state_d = `SET_CTRL;
                                next_task_state_d = `INIT_PSUM;
                            end
                            `OPCODE_WBPSRAM: state_d = `WRITE_BACK;
                            `OPCODE_WBPARAM: state_d = `WRITE_BACK_PARAM;
                            default:         state_d = `IDLE;
                        endcase
                    end else begin
                        state_d = `FETCH;
                    end
                end
                `PARAM_SET: begin
                    previous_state_d = `PARAM_SET;
                    state_d = `FETCH;
                end
                `WRITE_SRAM: begin
                    previous_state_d = `WRITE_SRAM;
                    state_d = (Param_TRG_q == `TRG_PSRAM) ? `CLEAR_CTRL : `FETCH;
                end
                `READ_SRAM: begin
                    previous_state_d = `READ_SRAM;
                    state_d = `FETCH;
                end
                `INIT_PSUM: begin
                    previous_state_d = `INIT_PSUM;
                    state_d = (psum_init_addr_q == `PE_ROW) ? `CLEAR_CTRL : `INIT_PSUM;
                end
                `SET: begin
                    previous_state_d = `SET;
                    state_d = `PRELOAD;
                end
                `PRELOAD: begin
                    previous_state_d = `PRELOAD;
                    state_d = `LOAD;
                end
                `LOAD: begin
                    previous_state_d = `LOAD;
                    state_d = (load_cnt_q == `PE_ROW-1) ? `RUN : `PRELOAD;
                end
                `RUN: begin
                    previous_state_d = `RUN;
                    if (run_cnt_q < (Param_S_q + `PE_ROW + `PE_COL)) begin
                        state_d = `RUN;
                    end else begin
                        if (Next_IC_q == 0 && Next_OC_q == 0) begin
                            state_d = `FETCH;
                        end else begin
                            state_d = `SET;
                        end
                    end
                end
                `WRITE_BACK: begin
                    previous_state_d = `WRITE_BACK;
                    state_d = `WRITE_BACK_READY;
                end
                `WRITE_BACK_READY: begin
                    previous_state_d = `WRITE_BACK_READY;
                    if (wb_ready_cnt_q < (WB_READY_WAIT_CYCLES - 1)) begin
                        state_d = `WRITE_BACK_READY;
                    end else begin
                        state_d = `WRITE_BACK_OUTPUT;
                    end
                end
                `WRITE_BACK_OUTPUT: begin
                    previous_state_d = `WRITE_BACK_READY;
                    if (wb_valid_cnt_q < (WB_VALID_HOLD_CYCLES - 1)) begin
                        state_d = `WRITE_BACK_OUTPUT;
                    end else begin
                        state_d = `FETCH;
                    end
                end
                `WRITE_BACK_PARAM: begin
                    previous_state_d = `WRITE_BACK_PARAM;
                    state_d = `FETCH;
                end
                `SET_CTRL: begin
                    previous_state_d = `SET_CTRL;
                    state_d = next_task_state_q;
                end
                `CLEAR_CTRL: begin
                    previous_state_d = `CLEAR_CTRL;
                    if (previous_state_q == `INIT_PSUM) begin
                        state_d = `SET;
                    end else begin
                        state_d = `FETCH;
                    end
                end
                default: state_d = `IDLE;
            endcase
            // 2. Output/Action Logic
            case (state_q)
                `FETCH: begin
                    // Fetch instruction and update outputs accordingly
                    Valid_WB_Out_d = 1'b0; // Clear valid flag
                    sram_input_we_d = 0; // Disable write to Input SRAM
                    sram_input_en_d = 0; // Disable Input SRAM
                    sram_weight_en_d = 0; // Disable Weight SRAM
                    sram_weight_we_d = 0; // Disable write to Weight SRAM
                    
                end
                `STABLIZE: begin
                    // if (instr_pulse) begin
                    //     instr_reg_d = Instr_In;
                    // end
                    instr_reg_d = Instr_In;

                end
                `DECODE: begin
                    opvalid_d = instr_reg_q[(`BIT_DATA + `BIT_PARAM + `BIT_OPCODE) +: `BIT_VALID]; // Extract valid bit from instruction
                    opcode_d = instr_reg_q[(`BIT_DATA + `BIT_PARAM) +: `BIT_OPCODE]; // Extract opcode from instruction
                    param_d = instr_reg_q[`BIT_DATA +: `BIT_PARAM]; // Extract param from instruction
                    sel_d = instr_reg_q[`BIT_DATA + `BIT_ADDR +: `BIT_SEL]; // Extract select bits from instruction
                    addr_d = instr_reg_q[`BIT_DATA +: `BIT_ADDR]; // Extract address from instruction
                    data_d = instr_reg_q[0 +: `BIT_DATA]; // Extract data from instruction
                end
                `EXECUTE: begin
                    if (opvalid_q) begin
                        case (opcode_q) 
                            `OPCODE_EX: begin
                                Flag_Finish_Out_d = 1'b0; // Clear finish flag
                            end
                            `OPCODE_WBPSRAM: begin
                                sel_d = sel_q;
                                addr_d = addr_q;
                            end
                        endcase
                    end
                end
                `PARAM_SET: begin
                    // Set parameters and update outputs accordingly
                    case (param_q)
                        `PARAM_S: begin
                            // Set S parameter
                            Param_S_d = data_q;
                        end
                        `PARAM_OC: begin
                            // Set OC parameter
                            Param_OC_d = data_q;
                        end
                        `PARAM_IC: begin
                            // Set IC parameter
                            Param_IC_d = data_q;
                        end
                        `PARAM_TRG: begin
                            // Set target for SRAM (Input, Weight, Psum)
                            Param_TRG_d = data_q;
                        end
                        `PARAM_BASE_WSRAM: begin
                            // Set base address for WSRAM
                            Param_BASE_WSRAM_d = data_q[`BIT_DATA-1:0];
                        end
                        `PARAM_BASE_WSRAM_WH: begin
                            // Set Higher bits of base address for WSRAM
                            Param_BASE_WSRAM_WH_d = data_q[`BIT_DATA-1:0];               
                        end
                        `PARAM_IC_WH: begin
                            // Set Higher bits of IC parameter
                            Param_IC_d = {data_q, Param_IC_q[`BIT_DATA-1:0]}; // Concatenate higher bits with existing IC parameter
                        end
                        default: begin 
                            // Default case, do nothing or reset outputs as needed 
                        end
                    endcase
                end
                `WRITE_SRAM: begin
                    // Write to SRAM and update outputs accordingly
                    case (Param_TRG_q)
                        `TRG_ISRAM: begin
                            // Write to Input SRAM
                            sram_input_we_d = 0; sram_input_we_d[isram_sel] = 1'b1; // Enable write to selected SRAM
                            sram_input_en_d = 0; sram_input_en_d[isram_sel] = 1'b1; // Enable selected SRAM
                            sram_input_addr_d[isram_sel*`BIT_ADDR+:`BIT_ADDR] = addr_q; // Address to write
                            sram_input_din_d[isram_sel*`BIT_DATA+:`BIT_DATA] = data_q; // Data to write
                        end
                        `TRG_WSRAM: begin
                            // Write to Weight SRAM
                            sram_weight_we_d = 0; sram_weight_we_d[wsram_sel] = 1'b1; // Enable write to selected SRAM
                            sram_weight_en_d = 0; sram_weight_en_d[wsram_sel] = 1'b1; // Enable selected SRAM
                            sram_weight_addr_d[wsram_sel*`BIT_ADDR+:`BIT_ADDR] = Base_W_full + addr_q;
                            sram_weight_din_d[wsram_sel*`BIT_DATA+:`BIT_DATA] = data_q; // Data to write
                        end
                        `TRG_PSRAM: begin
                            // Write to Psum SRAM (if applicable)
                            sram_psum_we_a_d = 0; sram_psum_we_a_d[psram_sel] = 1'b1; // Enable write to selected SRAM
                            sram_psum_en_a_d = 0; sram_psum_en_a_d[psram_sel] = 1'b1; // Enable selected SRAM
                            sram_psum_addr_a_d[psram_sel*`BIT_ADDR+:`BIT_ADDR] = addr_q; // Address to write
                            sram_psum_din_a_d[psram_sel*`BIT_DATA+:`BIT_DATA] = data_q; // Data to write
                        end
                        default: begin 
                            // Default case, do nothing or reset outputs as needed 
                        end
                    endcase
                end
                `INIT_PSUM: begin
                    // Initialize Psum SRAM
                    if (psum_init_addr_q == `PE_ROW) begin
                        sram_psum_we_a_d = 0; // Disable write to Psum SRAMs
                        sram_psum_en_a_d = 0; // Disable Psum SRAMs
                    end else begin
                        sram_psum_we_a_d = 4'b1111; // Enable write to all Psum SRAMs
                        sram_psum_en_a_d = 4'b1111; // Enable all Psum SRAMs
                    end
                    sram_psum_addr_a_d = {`PE_COL{psum_init_addr_q}}; // Address to write
                    sram_psum_din_a_d = {`PE_COL{{`BIT_PSUM{1'b0}}}}; // Data to write (zero initialization)
                    psum_init_addr_d = psum_init_addr_q + 1; // Increment address
                end
                `SET: begin
                    // Set up for execution and update outputs accordingly
                    // Reset load count and enable signals
                    load_cnt_d = 0; // Reset load count
                    run_cnt_d = 0; // Reset run count
                    // Load IC/OC status
                    State_IC_d = Next_IC_q;
                    State_OC_d = Next_OC_q;
                    // Determine Run_OC
                    if (Next_OC_q + `PE_COL <= Param_OC_q) begin
                        Run_OC_d = `PE_COL;
                    end else begin
                        Run_OC_d = Param_OC_q - Next_OC_q;
                    end
                    // Determine Run_IC
                    if (Next_IC_q + `PE_ROW <= Param_IC_q) begin
                        Run_IC_d = `PE_ROW;
                    end else begin
                        Run_IC_d = Param_IC_q - Next_IC_q;
                    end
                    // Next Index
                    if (Next_IC_q + `PE_ROW < Param_IC_q) begin
                        Next_IC_d = Next_IC_q + `PE_ROW;
                        Next_OC_d = Next_OC_q;
                    end else if (Next_OC_q + `PE_COL < Param_OC_q) begin
                        Next_IC_d = 0;
                        Next_OC_d = Next_OC_q + `PE_COL;
                    end else begin
                        Next_IC_d = 0;
                        Next_OC_d = 0;
                    end
                end
                `PRELOAD: begin
                    // 1. Enable Weight read
                    sram_weight_en_d = (Run_OC_q == 0) ? {`PE_COL{1'b0}} : ({`PE_COL{1'b1}} >> (`PE_COL - Run_OC_q));
                    for (i = 0; i < `PE_COL; i = i + 1) begin
                        if (i < Run_OC_q) begin
                            sram_weight_addr_d[i*`BIT_ADDR +: `BIT_ADDR] = Base_W_full + Base_W_calc + load_cnt_q;
                        end else begin
                            sram_weight_addr_d[i*`BIT_ADDR +: `BIT_ADDR] = {`BIT_ADDR{1'b0}};
                        end
                        $display("PRELOAD: Bank=%0d, Weight Addr = %0d", i, sram_weight_addr_d[i*`BIT_ADDR +: `BIT_ADDR]);
                        $display("PRELOAD DEBUG: load=%0d full=%0d calc=%0d (IC=%0d + ICstep=%0d * OCstep=%0d)", load_cnt_q, Base_W_full, Base_W_calc, State_IC_q, Param_IC_q, (State_OC_q / `PE_COL));
                    end
                end
                `LOAD: begin
                    // Load data and update outputs accordingly
                    load_cnt_d = load_cnt_q + 1; // Increment load count
                    // 2. Systolic Weight Write Control
                    systolic_en_row_id_d = load_cnt_q; // Set row ID
                    systolic_en_w_d = {`PE_COL{1'b1}}; // Enable for systolic array
                end
                `RUN: begin
                    // Execute and update outputs accordingly
                    sram_psum_en_b_d = 0; // Disable
                    systolic_valid_p_raw_d = 0; // Disable
                    //
                    if (run_cnt_q < (Param_S_q + `PE_ROW + `PE_COL)) begin
                        run_cnt_d = run_cnt_q + 1; // Increment run count
                    end else begin
                        if (Next_IC_q == 0 && Next_OC_q == 0) begin
                            Flag_Finish_Out_d = 1'b1; // Set finish flag
                        end else begin
                            Flag_Finish_Out_d = 1'b0; // Clear finish flag
                        end
                    end
                    // 1. Enable Input SRAM Read
                    $display("RUN: run_cnt=%0d", run_cnt_q);
                    if (run_cnt_q < Param_S_q) begin
                        sram_input_we_d = 0; // Disable write to Input SRAM
                        sram_input_en_d = (Run_IC_q == 0) ? {`PE_ROW{1'b0}} : ({`PE_ROW{1'b1}} >> (`PE_ROW - Run_IC_q));
                        for (i = 0; i < `PE_ROW; i = i + 1) begin
                            sram_input_addr_d[i*`BIT_ADDR+:`BIT_ADDR] = (i < Run_IC_q) ? (Base_I_calc + run_cnt_q) : {`BIT_ADDR{1'b0}};
                        end
                    end
                    // 2. Enable Psum SRAM Read
                    sram_psum_we_b_d = 0; // Disable write to Psum SRAM
                    sram_psum_en_b_d = (Run_OC_q == 0) ? {`PE_COL{1'b0}} : ({`PE_COL{1'b1}} >> (`PE_COL - Run_OC_q));
                    for (i = 0; i < `PE_COL; i = i + 1) begin
                        sram_psum_addr_b_d[i*`BIT_ADDR+:`BIT_ADDR] = (i < Run_OC_q) ? (((State_OC_q/`PE_COL)*Param_S_q) + pattern_array[i]) : {`BIT_ADDR{1'b0}};
                    end
                    // 3. Systolic Psum Write Control
                    for (i = 0; i < `PE_COL; i = i + 1) begin
                        systolic_addr_p_d[i*`BIT_ADDR+:`BIT_ADDR] = (i < Run_OC_q) ? (((State_OC_q/`PE_COL)*Param_S_q) + pattern_array_d[i]) : {`BIT_ADDR{1'b0}};
                    end
                    // 4. Set Psum Valid bits based on run_cnt
                                    // Previous Logic
                                    // if (run_cnt_q < `PE_COL) begin
                                    //     systolic_valid_p_raw_d = (1 << (run_cnt_q + 1)) - 1; // Enable valid bits
                                    // end else if (run_cnt_q >= `PE_COL && run_cnt_q < 2 * `PE_COL - 1) begin
                                    //     systolic_valid_p_raw_d = ((1 << (2 * `PE_COL - run_cnt_q - 1)) - 1) << (run_cnt_q - `PE_COL + 1); // Shift valid bits
                                    // end else begin
                                    //     systolic_valid_p_raw_d = 0; // Disable valid bits
                                    // end
                    // for (i = 0; i < `PE_COL; i = i + 1) begin
                    //     if ((i >= (run_cnt_q - (Param_S_q - 1))) && (i <= run_cnt_q) && (i < `PE_COL))
                    //         systolic_valid_p_raw_d[i] = 1'b1;
                    //     else
                    //         systolic_valid_p_raw_d[i] = 1'b0;
                    // if (run_cnt_q == 0)
                    //     systolic_valid_p_raw_d = 1'b1;
                    // end
                    // if (run_cnt_q >= (Param_S_q + `PE_COL - 1))
                    //     systolic_valid_p_raw_d = 0;
                    // 4-1. Calculate window start (clamped to 0)
                    if (run_cnt_q >= Param_S_q -1)
                        win_start = run_cnt_q - (Param_S_q - 1);
                    else
                        win_start = 0;
                    // 4-2. Calculate window end (clamped to PE_COL-1)
                    win_end = run_cnt_q;
                    if (win_end >= `PE_COL)
                        win_end = `PE_COL - 1;
                    // 4-3. Apply window
                    for (i = 0; i < `PE_COL; i = i + 1) begin
                        if (i >= win_start && i <= win_end)
                            systolic_valid_p_raw_d[i] = 1'b1;
                        else
                            systolic_valid_p_raw_d[i] = 1'b0;
                    end

                    // 5. Masking
                    oc_mask = (Run_OC_q == 0) ? {`PE_COL{1'b0}} : ({`PE_COL{1'b1}} >> (`PE_COL - Run_OC_q));
                    systolic_valid_p_d = systolic_valid_p_raw_q & oc_mask; // Apply mask
                end
                `WRITE_BACK: begin
                    // Write back results and update outputs accordingly
                    sram_psum_we_b_d = {`PE_COL{1'b0}}; // Disable write to Psum SRAM
                    sram_psum_en_b_d = 0; sram_psum_en_b_d[psram_sel] = 1'b1; // Enable selected Psum SRAM
                    sram_psum_addr_b_d[psram_sel*`BIT_ADDR+:`BIT_ADDR] = addr_q; // Address to read
                    // Flag_Finish_Out_d = 1'b0; // Clear finish flag
                    wb_ready_cnt_d = {WB_READY_CNT_WIDTH{1'b0}}; // Reset WB ready wait counter
                    psum_sel_ctrl_d = 1'b1; // Enable Psum SRAM control
                    wb_valid_cnt_d = {WB_CNT_WIDTH{1'b0}}; // Reset WB valid hold counter
                    Valid_WB_Out_d = 1'b0; // Clear valid flag
                end
                `WRITE_BACK_READY: begin
                    // BRAM latency 기다리는 동안 EN경로 유지
                    psum_sel_ctrl_d = 1'b1; // Keep Psum SRAM control enabled
                    Valid_WB_Out_d = 1'b0;
                    // Flag_Finish_Out_d = 1'b0; // Clear finish flag

                    // (A) B 포트 Write 금지
                    sram_psum_we_b_d = {`PE_COL{1'b0}}; // Ensure write is disabled

                    // (B) lane 선택/enable -> sel_q 비트만 1
                    sram_psum_en_b_d = {`PE_COL{1'b0}};
                    sram_psum_en_b_d[psram_sel] = 1'b1;
                    //sram_psum_en_b_d = sram_psum_en_b_q; // Maintain previous enable state

                    // (C) 주소 셋업 -> 해당 lane에 addr_q 반영
                    sram_psum_addr_b_d[psram_sel*`BIT_ADDR +: `BIT_ADDR] = addr_q;

                    // Latency Count            
                    if (wb_ready_cnt_q < (WB_READY_WAIT_CYCLES - 1)) begin
                        wb_ready_cnt_d = wb_ready_cnt_q + 1'b1; // Increment WB ready wait counter
                    end
                end
                `WRITE_BACK_OUTPUT: begin
                    // Write back output data and update outputs accordingly
                    psum_sel_ctrl_d = 1'b1; // Keep Psum SRAM control enabled
                    // Flag_Finish_Out_d = 1'b0; // Clear finish flag

                    // Valid_WB_Out_d = 1'b1; // Set valid flag


                    // 첫 사이클에만 샘플, 이후엔 유지
                    if (wb_valid_cnt_q == 0) begin
                        Data_WB_Out_d = sram_psum_dout_b[psram_sel*`BIT_PSUM+:`BIT_PSUM]; // Sample output data from Psum SRAM
                        Valid_WB_Out_d = 1'b1;
                    end else begin
                        Data_WB_Out_d = Data_WB_Out_q; // Hold previous output data
                        Valid_WB_Out_d = 1'b1;
                    end

                    // 이제 EN 내려도 안전
                    sram_psum_en_b_d = {`PE_COL{1'b0}}; // Disable Psum SRAM enable
                    sram_psum_we_b_d = {`PE_COL{1'b0}}; // Ensure write is disabled

                    // 카운터 증가 (최대 WB_VALID_HOLD_CYCLES-1까지만)
                    if (wb_valid_cnt_q < (WB_VALID_HOLD_CYCLES - 1)) begin
                        wb_valid_cnt_d = wb_valid_cnt_q + 1'b1;
                    end
                end
                `WRITE_BACK_PARAM: begin
                    // Write back parameters and update outputs accordingly
                    case(param_q)
                        `PARAM_BASE_WSRAM: begin
                            // Write back base address for WSRAM
                            Data_WB_Out_d = Base_W_full; // Output base address
                            Valid_WB_Out_d = 1'b1; // Set valid flag
                        end
                        `PARAM_S: begin
                            // Write back S parameter
                            Data_WB_Out_d = Param_S_q; // Output S parameter
                            Valid_WB_Out_d = 1'b1; // Set valid flag
                        end
                        `PARAM_OC: begin
                            // Write back OC parameter
                            Data_WB_Out_d = Param_OC_q; // Output OC parameter
                            Valid_WB_Out_d = 1'b1; // Set valid flag
                        end
                        `PARAM_IC: begin
                            // Write back IC parameter
                            Data_WB_Out_d = Param_IC_q; // Output IC parameter
                            Valid_WB_Out_d = 1'b1; // Set valid flag
                        end
                        `PARAM_TRG: begin
                            // Write back target for SRAM
                            Data_WB_Out_d = Param_TRG_q; // Output target
                            Valid_WB_Out_d = 1'b1; // Set valid flag
                        end
                        default: begin
                            // Default case, do nothing or reset outputs as needed
                        end
                    endcase
                end
                `SET_CTRL: begin
                    psum_sel_ctrl_d = 1'b1; // Enable Psum SRAM control
                end
                `CLEAR_CTRL: begin
                    psum_sel_ctrl_d = 1'b0; // Disable Psum SRAM control
                end
                default: begin
                    // Default case, do nothing or reset outputs as needed
                end
            endcase
        end
    // # endregion  

    // 7) Sequential Block : *_q <= *_d
    // # region Sequential Block
        always @(posedge CLK or negedge RSTb) begin
            if (!RSTb) begin
            // 비동기 리셋(모든 Q 초기화)
                // 상태
                state_q <= `IDLE;
                previous_state_q <= `IDLE;
                next_task_state_q <= `IDLE;

                // Decode 결과
                opvalid_q <= 1'b0;
                opcode_q <= 0;
                param_q <= 0;
                sel_q <= 0;
                addr_q <= 0;
                data_q <= 0;

                // 파라미터 / 인덱스
                Param_S_q <= 0;
                Param_OC_q <= 0;
                Param_IC_q <= 0;
                Param_TRG_q <= 0;

                State_IC_q <= 0;
                State_OC_q <= 0;
                Next_IC_q <= 0;
                Next_OC_q <= 0;
                Run_IC_q <= 0;
                Run_OC_q <= 0;

                // 카운터
                psum_init_addr_q <= 0;
                load_cnt_q <= 0;
                run_cnt_q <= 0;
                clk_cnt_q <= 0;

                // SRAM / Systolic 제어신호
                sram_input_we_q <= 0;
                sram_input_en_q <= 0;
                sram_input_addr_q <= 0;
                sram_input_din_q <= 0;

                sram_weight_we_q <= 0;
                sram_weight_en_q <= 0;
                sram_weight_addr_q <= 0;
                sram_weight_din_q <= 0;

                sram_psum_we_a_q <= 0;
                sram_psum_en_a_q <= 0;
                sram_psum_addr_a_q <= 0;
                sram_psum_din_a_q <= 0;

                sram_psum_we_b_q <= 0;
                sram_psum_en_b_q <= 0;
                sram_psum_addr_b_q <= 0;

                systolic_en_row_id_q <= 0; 
                systolic_en_w_q <= 0;
                systolic_addr_p_q <= 0;
                systolic_valid_p_q <= 0;
                systolic_valid_p_raw_q <= 0;

                psum_sel_ctrl_q <= 0;

                // WriteBack
                Flag_Finish_Out_q <= 0;
                Valid_WB_Out_q <= 0;
                Data_WB_Out_q <= 0;  
                wb_valid_cnt_q <= 0; 

                wb_ready_cnt_q <= 0;
                instr_reg_q <= 0;
                Param_BASE_WSRAM_q <= 0;
                Param_BASE_WSRAM_WH_q <= 0;

            end else begin
                // 동기식 상태 갱신
                Flag_Finish_Out_q <= Flag_Finish_Out_d;
                Valid_WB_Out_q <= Valid_WB_Out_d;
                Data_WB_Out_q <= Data_WB_Out_d;
                sram_input_we_q <= sram_input_we_d;
                sram_input_en_q <= sram_input_en_d;
                sram_input_addr_q <= sram_input_addr_d;
                sram_input_din_q <= sram_input_din_d;
                sram_weight_we_q <= sram_weight_we_d;
                sram_weight_en_q <= sram_weight_en_d;
                sram_weight_addr_q <= sram_weight_addr_d;
                sram_weight_din_q <= sram_weight_din_d;
                sram_psum_we_a_q <= sram_psum_we_a_d;
                sram_psum_en_a_q <= sram_psum_en_a_d;
                sram_psum_addr_a_q <= sram_psum_addr_a_d;
                sram_psum_din_a_q <= sram_psum_din_a_d;
                sram_psum_we_b_q <= sram_psum_we_b_d;
                sram_psum_en_b_q <= sram_psum_en_b_d;
                sram_psum_addr_b_q <= sram_psum_addr_b_d;
                systolic_en_row_id_q <= systolic_en_row_id_d;
                systolic_en_w_q <= systolic_en_w_d;
                systolic_addr_p_q <= systolic_addr_p_d;
                systolic_valid_p_q <= systolic_valid_p_d;
                psum_sel_ctrl_q <= psum_sel_ctrl_d;
                state_q <= state_d;
                previous_state_q <= previous_state_d;
                next_task_state_q <= next_task_state_d;
                opvalid_q <= opvalid_d;
                opcode_q <= opcode_d;
                param_q <= param_d;
                sel_q <= sel_d;
                addr_q <= addr_d;
                data_q <= data_d;
                Param_S_q <= Param_S_d;
                Param_OC_q <= Param_OC_d;
                Param_IC_q <= Param_IC_d;
                Param_TRG_q <= Param_TRG_d;
                psum_init_addr_q <= psum_init_addr_d;
                State_IC_q <= State_IC_d;
                State_OC_q <= State_OC_d;
                Next_IC_q <= Next_IC_d;
                Next_OC_q <= Next_OC_d;
                Run_IC_q <= Run_IC_d;
                Run_OC_q <= Run_OC_d;
                load_cnt_q <= load_cnt_d;
                run_cnt_q <= run_cnt_d;
                clk_cnt_q <= clk_cnt_d;
                systolic_valid_p_raw_q <= systolic_valid_p_raw_d;
                wb_valid_cnt_q <= wb_valid_cnt_d;
                wb_ready_cnt_q <= wb_ready_cnt_d;
                instr_reg_q <= instr_reg_d;

                Param_BASE_WSRAM_q <= Param_BASE_WSRAM_d;
                Param_BASE_WSRAM_WH_q <= Param_BASE_WSRAM_WH_d;

            end
        end
    // # endregion

    // 8) Pattern Array
    // # region Pattern Array
        always @(*) begin
            case (run_cnt_q)
                0: begin
                    pattern_array[0] = 0;
                    pattern_array[1] = 0;
                    pattern_array[2] = 0;
                    pattern_array[3] = 0;
                end
                1: begin
                    pattern_array[0] = 1;
                    pattern_array[1] = 0;
                    pattern_array[2] = 0;
                    pattern_array[3] = 0;
                end
                2: begin
                    pattern_array[0] = 2;
                    pattern_array[1] = 1;
                    pattern_array[2] = 0;
                    pattern_array[3] = 0;
                end
                3: begin
                    pattern_array[0] = 3;
                    pattern_array[1] = 2;
                    pattern_array[2] = 1;
                    pattern_array[3] = 0;
                end
                4: begin
                    pattern_array[0] = 0;
                    pattern_array[1] = 3;
                    pattern_array[2] = 2;
                    pattern_array[3] = 1;
                end
                5: begin
                    pattern_array[0] = 0;
                    pattern_array[1] = 0;
                    pattern_array[2] = 3;
                    pattern_array[3] = 2;
                end
                6: begin
                    pattern_array[0] = 0;
                    pattern_array[1] = 0;
                    pattern_array[2] = 0;
                    pattern_array[3] = 3;
                end
                7: begin
                    pattern_array[0] = 0;
                    pattern_array[1] = 0;
                    pattern_array[2] = 0;
                    pattern_array[3] = 0;
                end
                8: begin
                    pattern_array[0] = 0;
                    pattern_array[1] = 0;
                    pattern_array[2] = 0;
                    pattern_array[3] = 0;
                end
                default: begin
                    pattern_array[0] = 0;
                    pattern_array[1] = 0;
                    pattern_array[2] = 0;
                    pattern_array[3] = 0;
                end
            endcase
        end
        always @(*) begin
            case (run_cnt_q)
                0: begin
                    pattern_array_d[0] = 0;
                    pattern_array_d[1] = 0;
                    pattern_array_d[2] = 0;
                    pattern_array_d[3] = 0;
                end                
                1: begin
                    pattern_array_d[0] = 0;
                    pattern_array_d[1] = 0;
                    pattern_array_d[2] = 0;
                    pattern_array_d[3] = 0;
                end
                2: begin
                    pattern_array_d[0] = 1;
                    pattern_array_d[1] = 0;
                    pattern_array_d[2] = 0;
                    pattern_array_d[3] = 0;
                end
                3: begin
                    pattern_array_d[0] = 2;
                    pattern_array_d[1] = 1;
                    pattern_array_d[2] = 0;
                    pattern_array_d[3] = 0;
                end
                4: begin
                    pattern_array_d[0] = 3;
                    pattern_array_d[1] = 2;
                    pattern_array_d[2] = 1;
                    pattern_array_d[3] = 0;
                end
                5: begin
                    pattern_array_d[0] = 0;
                    pattern_array_d[1] = 3;
                    pattern_array_d[2] = 2;
                    pattern_array_d[3] = 1;
                end
                6: begin
                    pattern_array_d[0] = 0;
                    pattern_array_d[1] = 0;
                    pattern_array_d[2] = 3;
                    pattern_array_d[3] = 2;
                end
                7: begin
                    pattern_array_d[0] = 0;
                    pattern_array_d[1] = 0;
                    pattern_array_d[2] = 0;
                    pattern_array_d[3] = 3;
                end
                8: begin
                    pattern_array_d[0] = 0;
                    pattern_array_d[1] = 0;
                    pattern_array_d[2] = 0;
                    pattern_array_d[3] = 0;
                end
                default: begin
                    pattern_array_d[0] = 0;
                    pattern_array_d[1] = 0;
                    pattern_array_d[2] = 0;
                    pattern_array_d[3] = 0;
                end
            endcase
        end
    // # endregion

    // 9) Debug Signals
    // # region Debug Signals
        assign state_out = state_q[`BIT_STATE-1:0];
        assign sram_weight_addr_out = sram_weight_addr_q;
        assign sram_weight_din_out = sram_weight_din_q;
    // # endregion

endmodule