// Copyright 2020 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Florian Glaser <glaserf@iis.ee.ethz.ch>

module afe_top #(
  parameter AFE_RX_TYPE       = 0,

  parameter L2_AWIDTH_NOAL    = 12,
  parameter L2_TRANS_SIZE     = 16,
  parameter L2_NUM_CHS        = 8,

  parameter BUFF_AWIDTH       = 10,
  parameter BUFF_TRANS_SIZE   = 16,

  parameter AFE_DATA_WIDTH    = 32,
  parameter AFE_NUM_CHS       = 8,
  parameter AFE_PL_WIDTH      = 16,
  parameter AFE_CHID_LSB      = 28,
  parameter AFE_CHID_WIDTH    = 4,
  parameter AFE_SUBCHID_LSB   = 28,
  parameter AFE_SUBCHID_WIDTH = 4,
  parameter AFE_FLAG_LSB      = 16,
  parameter AFE_FLAG_WIDTH    = 4,

  parameter FEATURE_FLAG      = 0
) (
  input  logic                        clk_i,      
  input  logic                        rst_ni,
  input  logic                        test_mode_i,

  input  logic                        intf_shtdwn_i,

  input  logic                 [31:0] cfg_wdata_i,
  input  logic                 [10:0] cfg_addr_i,
  input  logic                        cfg_valid_i,
  input  logic                        cfg_rwn_i,
  output logic                 [31:0] cfg_rdata_o,
  output logic                        cfg_ready_o,

  output logic       [L2_NUM_CHS-1:0] l2_ch_event_o,
  output logic                        buff_event_o,
  output logic                        flag_event_o,

  output logic      [BUFF_AWIDTH-1:0] buff_waddr_o,
  output logic      [BUFF_AWIDTH-1:0] buff_raddr_o,

  input  logic                        buff_rvalid_i,
  input  logic   [AFE_DATA_WIDTH-1:0] buff_rdata_i, // for local field extraction

  output logic   [L2_AWIDTH_NOAL-1:0] l2_addr_o,
  output logic                  [1:0] l2_size_o,
  output logic                 [31:0] l2_wdata_o,

  output logic                        flag_valid_o,
  input  logic                        flag_ready_i,
  output logic                 [31:0] flag_data_o,

  /* Handshake signals */
  input  logic                        buff_wr_ready_i, // write access to buffer granted
  output logic                        buff_rd_valid_o, // read access to buffer request
  input  logic                        buff_rd_ready_i, // read access to buffer granted

  output logic                        afe_valid_o,
  output logic   [AFE_DATA_WIDTH-1:0] afe_data_o,

  /* AFE data and valid, non-stallable */
  input  logic                        afe_valid_async_i,
  input  logic   [AFE_DATA_WIDTH-1:0] afe_data_async_i
);

  localparam FLAG_CNT_WIDTH = 6;

  // L2 channels
  logic [L2_NUM_CHS-1:0]   [L2_AWIDTH_NOAL-1:0] l2_ch_startaddr;
  logic [L2_NUM_CHS-1:0]    [L2_TRANS_SIZE-1:0] l2_ch_size;
  logic [L2_NUM_CHS-1:0]                  [1:0] l2_ch_datasize;
  logic [L2_NUM_CHS-1:0]                        l2_ch_continuous;
  logic [L2_NUM_CHS-1:0]                        l2_ch_cfg_en;
  logic [L2_NUM_CHS-1:0]                        l2_ch_clr;
  logic [L2_NUM_CHS-1:0]                        l2_ch_en;
  logic [L2_NUM_CHS-1:0][AFE_SUBCHID_WIDTH-1:0] l2_ch_subchid;
  logic [L2_NUM_CHS-1:0]   [AFE_CHID_WIDTH-1:0] l2_ch_chid;
  logic [L2_NUM_CHS-1:0]   [L2_AWIDTH_NOAL-1:0] l2_ch_curr_addr;
  logic [L2_NUM_CHS-1:0]    [L2_TRANS_SIZE-1:0] l2_ch_wr_ptr;
  logic [L2_NUM_CHS-1:0]    [L2_TRANS_SIZE-1:0] l2_ch_bytes_left;

  logic [L2_NUM_CHS-1:0]                        l2_ch_tf_valid;
  /* for types 2 and 3 only */
  logic [$clog2(AFE_NUM_CHS)-1:0]               l2_ch_chid_match;

  // Buffer
  logic     [BUFF_AWIDTH-1:0] buff_startaddr;
  logic [BUFF_TRANS_SIZE-1:0] buff_size;
  logic [BUFF_TRANS_SIZE-1:0] buff_flevel;
  logic                       buff_continuous;
  logic                       buff_ovflw_mode;
  logic                       buff_cfg_clr;
  logic                       buff_cfg_en;
  logic                       buff_en;
  logic     [BUFF_AWIDTH-1:0] buff_wr_addr;
  logic     [BUFF_AWIDTH-1:0] buff_rd_addr;
  logic [BUFF_TRANS_SIZE-1:0] buff_bytes_left;

  // AFE input masking
  logic     [AFE_NUM_CHS-1:0] afe_ch_mask;
  logic                       afe_ch_en_mode;
  logic  [AFE_CHID_WIDTH-1:0] afe_ch_en_chid;

  // includes sign extension
  logic                 [1:0] data_mask_mode;

  // clock gating of l2 channels
  logic                       clk_l2_side;

  // Flags
  logic                       flag_en;
  logic  [AFE_FLAG_WIDTH-1:0] flag_mask;
  logic                [31:0] flag_data_q, flag_data_n;
  logic                       flag_valid_q, flag_valid_n;
  logic                       flag_tf_valid;

  logic                         flag_event_clr;
  logic                         flag_event_q, flag_event_n;
  logic    [FLAG_CNT_WIDTH-1:0] flag_cnt_q, flag_cnt_n;
  enum logic {IDLE, TRIGGERED}  flag_state_q, flag_state_n;


  /* extracted data and meta fields of afe data */
  logic      [AFE_PL_WIDTH-1:0] rdata_payload;
  logic    [AFE_CHID_WIDTH-1:0] rdata_chid;
  logic [AFE_SUBCHID_WIDTH-1:0] rdata_subchid;
  logic    [AFE_FLAG_WIDTH-1:0] rdata_flags;


  /* assignments */
  assign buff_waddr_o  = buff_wr_addr;
  assign buff_raddr_o  = buff_rd_addr;

  assign flag_valid_o  = flag_valid_q;
  assign flag_data_o   = flag_data_q;
  assign flag_event_o  = flag_event_q;

  assign rdata_payload = buff_rvalid_i ? buff_rdata_i[0            +: AFE_PL_WIDTH]   : 0;
  assign rdata_chid    = buff_rvalid_i ? buff_rdata_i[AFE_CHID_LSB +: AFE_CHID_WIDTH] : 0;
  assign rdata_flags   = buff_rvalid_i ? buff_rdata_i[AFE_FLAG_LSB +: AFE_FLAG_WIDTH] : 0;

  if ((AFE_RX_TYPE == 1) || (AFE_RX_TYPE == 3))
    assign rdata_subchid = buff_rvalid_i ? buff_rdata_i[AFE_SUBCHID_LSB +: AFE_SUBCHID_WIDTH] : 0;
  else
    assign rdata_subchid = '0;

  /* L2 address, transfer size and data muxing */
  assign l2_size_o     = buff_rvalid_i ? l2_ch_datasize[rdata_chid]  : 0;

  /* AFE type-dependent assignments */
  case(AFE_RX_TYPE)
    0: assign l2_addr_o = buff_rvalid_i ? l2_ch_curr_addr[rdata_chid      ]                                        : 32'hBADACCE5;
    1: assign l2_addr_o = buff_rvalid_i ? l2_ch_curr_addr[rdata_chid      ] + rdata_subchid*l2_ch_size[rdata_chid] : 32'hBADACCE5;
    2: assign l2_addr_o = buff_rvalid_i ? l2_ch_curr_addr[l2_ch_chid_match]                                        : 32'hBADACCE5;
    3: assign l2_addr_o = buff_rvalid_i ? l2_ch_curr_addr[l2_ch_chid_match] + rdata_subchid*l2_ch_size[rdata_chid] : 32'hBADACCE5;
  
    default: assign l2_addr_o = 32'hBADACCE5;
  endcase

  if ((AFE_RX_TYPE == 2) || (AFE_RX_TYPE == 3)) begin
    always_comb begin
      l2_ch_chid_match = '0;

      for (int i=0; i<L2_NUM_CHS; i++) begin
        if (buff_rvalid_i && (rdata_chid == l2_ch_chid[i])) begin
          l2_ch_chid_match = i;
          break;
        end
      end
    end
  end
  else begin
    assign l2_ch_chid_match = '0;
  end

  always_comb begin
    l2_wdata_o = '0;

    if (buff_rvalid_i) begin
      case (data_mask_mode)
        2'b00: l2_wdata_o = buff_rdata_i;
        2'b01: begin
          l2_wdata_o[0            +: AFE_PL_WIDTH]   = rdata_payload;
          l2_wdata_o[AFE_FLAG_LSB +: AFE_FLAG_WIDTH] = rdata_flags;
        end
        2'b10: begin
          l2_wdata_o[0            +: AFE_PL_WIDTH]   = rdata_payload;
        end
        2'b11: begin /* sign-extended payload data */
          l2_wdata_o[AFE_PL_WIDTH-1:0] = rdata_payload;
          l2_wdata_o[31:AFE_PL_WIDTH]  = {(32-AFE_PL_WIDTH){rdata_payload[AFE_PL_WIDTH-1]}};
        end
        default: l2_wdata_o = buff_rdata_i;
      endcase
    end
  end


  /* flag generation */
  localparam FLAG_SUBCHID_LSB = 24 + AFE_CHID_WIDTH;
  localparam FLAG_SUBCHID_MSB = FLAG_SUBCHID_LSB + AFE_SUBCHID_WIDTH - 1;

  if (FEATURE_FLAG) begin
    assign flag_tf_valid = flag_valid_q & flag_ready_i;

    always_comb begin
      flag_valid_n = flag_valid_q;
      flag_data_n  = flag_data_q;

      if (flag_en & |(rdata_flags & flag_mask)) begin
        flag_data_n[31  : FLAG_SUBCHID_MSB+1]          = '0;
        flag_data_n[FLAG_SUBCHID_MSB:FLAG_SUBCHID_LSB] = rdata_subchid;

        flag_data_n[24 +: AFE_CHID_WIDTH]    = rdata_chid;
        flag_data_n[23  : 16+AFE_FLAG_WIDTH] = '0;
        flag_data_n[16 +: AFE_FLAG_WIDTH]    = rdata_flags;
        flag_data_n[15  : L2_TRANS_SIZE]     = '0;
        flag_data_n[0  +: L2_TRANS_SIZE]     = l2_ch_wr_ptr[rdata_chid];

        flag_valid_n = 1'b1;
      end
      else if (flag_valid_q & flag_ready_i) begin
        flag_valid_n = 1'b0;
      end
    end

    always_comb begin
      flag_state_n = flag_state_q;
      flag_event_n = 1'b0;
      flag_cnt_n   = flag_cnt_q;

      case(flag_state_q)
        IDLE: begin
          if (flag_tf_valid) begin
            flag_event_n = 1'b1;
            flag_cnt_n   = 'd1;
            flag_state_n = TRIGGERED;
          end
        end

        TRIGGERED: begin
          if (flag_event_clr & ~flag_tf_valid) begin
            flag_cnt_n   = '0;
            flag_state_n = IDLE;
          end
          else if (flag_event_clr & flag_tf_valid) begin
            flag_cnt_n   = 'd1;
            flag_event_n = 1'b1;
          end
          else if (flag_tf_valid) begin
            flag_cnt_n   = (flag_cnt_q == '1) ? '0 : flag_cnt_q + 1;
          end
        end
      endcase
    end
  end
  else begin
    /* dummy assignments if flag feature not present */
    assign flag_tf_valid = 1'b0;
    assign flag_valid_n  = 1'b0;
    assign flag_data_n   = '0;
    assign flag_cnt_n    = '0;
    assign flag_event_n  = 1'b0;
    assign flag_state_n  = IDLE;
  end


  afe_sync_if #(
    .AFE_DATA_WIDTH ( AFE_DATA_WIDTH ),
    .AFE_NUM_CHS    ( AFE_NUM_CHS    ),
    .AFE_CHID_LSB   ( AFE_CHID_LSB   ),
    .AFE_CHID_WIDTH ( AFE_CHID_WIDTH )
  )
  afe_sync_if_i (
    .clk_i,
    .rst_ni,
    .test_mode_i,

    .afe_ch_mask_i      ( afe_ch_mask     ),
    .afe_ch_en_mode_i   ( afe_ch_en_mode  ),
    .afe_ch_en_chid_i   ( afe_ch_en_chid  ),

    .buff_cfg_en_i      ( buff_cfg_en     ),
    .buff_cfg_clr_i     ( buff_cfg_clr    ),

    .afe_valid_async_i,
    .afe_data_async_i,
    .afe_valid_sync_o   ( afe_valid_o     ),
    .afe_data_sync_o    ( afe_data_o      ),

    .buff_ready_i       ( buff_wr_ready_i )
  );


  afe_reg_if #(
    .AFE_RX_TYPE       ( AFE_RX_TYPE       ),
    .L2_AWIDTH_NOAL    ( L2_AWIDTH_NOAL    ),
    .L2_TRANS_SIZE     ( L2_TRANS_SIZE     ),
    .L2_NUM_CHS        ( L2_NUM_CHS        ),
    .AFE_NUM_CHS       ( AFE_NUM_CHS       ),
    .AFE_CHID_WIDTH    ( AFE_CHID_WIDTH    ),
    .AFE_SUBCHID_WIDTH ( AFE_SUBCHID_WIDTH ),
    .AFE_FLAG_WIDTH    ( AFE_FLAG_WIDTH    ),
    .BUFF_AWIDTH       ( BUFF_AWIDTH       ),
    .BUFF_TRANS_SIZE   ( BUFF_TRANS_SIZE   ),
    .FLAG_CNT_WIDTH    ( FLAG_CNT_WIDTH    )
  )
  afe_reg_if_i (
    .clk_i,
    .rst_ni,
    .test_mode_i,

    .cfg_wdata_i,
    .cfg_addr_i,
    .cfg_valid_i,
    .cfg_rwn_i,
    .cfg_rdata_o,
    .cfg_ready_o,

    // Flag configuration and control
    .cfg_flag_en_o         ( flag_en          ),
    .cfg_flag_mask_o       ( flag_mask        ),
    .cfg_flag_data_i       ( flag_data_q      ),
    .cfg_flag_cnt_i        ( flag_cnt_q       ),
    .cfg_flag_clr_o        ( flag_event_clr   ),

    // L2 channels configuration and status
    .cfg_l2_startaddr_o    ( l2_ch_startaddr  ),
    .cfg_l2_size_o         ( l2_ch_size       ),
    .cfg_l2_datasize_o     ( l2_ch_datasize   ),
    .cfg_l2_continuous_o   ( l2_ch_continuous ),
    .cfg_l2_en_o           ( l2_ch_cfg_en     ),
    .cfg_l2_clr_o          ( l2_ch_clr        ),
    .cfg_l2_subchid_o      ( l2_ch_subchid    ),
    .cfg_l2_chid_o         ( l2_ch_chid       ),
    .cfg_l2_en_i           ( l2_ch_en         ),
    .cfg_l2_curr_addr_i    ( l2_ch_curr_addr  ),
    .cfg_l2_bytes_left_i   ( l2_ch_bytes_left ),

    // Buffer channel configuration and status
    .cfg_buff_startaddr_o  ( buff_startaddr   ),
    .cfg_buff_size_o       ( buff_size        ),
    .cfg_buff_continuous_o ( buff_continuous  ),
    .cfg_buff_overflow_o   ( buff_ovflw_mode  ),
    .cfg_buff_en_o         ( buff_cfg_en      ),
    .cfg_buff_clr_o        ( buff_cfg_clr     ),
    .cfg_buff_flevel_o     ( buff_flevel      ),
    .cfg_buff_en_i         ( buff_en          ),
    .cfg_buff_curr_waddr_i ( buff_wr_addr     ),
    .cfg_buff_curr_raddr_i ( buff_rd_addr     ),
    .cfg_buff_bytes_left_i ( buff_bytes_left  ),

    // Misc control and config
    .cfg_afe_ch_mask_o     ( afe_ch_mask      ),
    .cfg_afe_ch_en_mode_o  ( afe_ch_en_mode   ),
    .cfg_afe_ch_en_chid_o  ( afe_ch_en_chid   ),
    .cfg_data_mask_mode_o  ( data_mask_mode   )
  );


  for(genvar I=0; I<L2_NUM_CHS; I++) begin
    case(AFE_RX_TYPE)
      0: assign l2_ch_tf_valid[I] = buff_rvalid_i ? (rdata_chid       == I)                                        : 1'b0;
      1: assign l2_ch_tf_valid[I] = buff_rvalid_i ? (rdata_chid       == I) && (rdata_subchid == l2_ch_subchid[I]) : 1'b0;
      2: assign l2_ch_tf_valid[I] = buff_rvalid_i ? (l2_ch_chid_match == I)                                        : 1'b0;
      3: assign l2_ch_tf_valid[I] = buff_rvalid_i ? (l2_ch_chid_match == I) && (rdata_subchid == l2_ch_subchid[I]) : 1'b0;

      default: assign l2_ch_tf_valid[I] = 1'b0;
    endcase

    afe_l2_addrgen #(
      .AWIDTH       ( L2_AWIDTH_NOAL ),
      .TRANS_SIZE   ( L2_TRANS_SIZE  )
    ) 
    afe_l2_addrgen_i (
      .clk_i             ( clk_l2_side         ),
      .rst_ni,
      .test_mode_i,

      .cfg_startaddr_i   ( l2_ch_startaddr[I]  ),
      .cfg_size_i        ( l2_ch_size[I]       ),
      .cfg_datasize_i    ( l2_ch_datasize[I]   ),
      .cfg_continuous_i  ( l2_ch_continuous[I] ),
      .cfg_en_i          ( l2_ch_cfg_en[I]     ),
      .cfg_clr_i         ( l2_ch_clr[I]        ),
      .cfg_curr_addr_o   ( l2_ch_curr_addr[I]  ),
      .cfg_wr_ptr_o      ( l2_ch_wr_ptr[I]     ),
      .cfg_bytes_left_o  ( l2_ch_bytes_left[I] ),
      .cfg_en_o          ( l2_ch_en[I]         ),

      .event_o           ( l2_ch_event_o[I]    ),

      .transfer_valid_i  ( l2_ch_tf_valid[I]   )
    );
  end


  afe_buff_addrgen #(
    .AWIDTH     ( BUFF_AWIDTH     ),
    .TRANS_SIZE ( BUFF_TRANS_SIZE )
  )
  afe_buff_addrgen_i (
    .clk_i,
    .rst_ni,
    .test_mode_i,

    .cfg_startaddr_i  ( buff_startaddr  ),
    .cfg_size_i       ( buff_size       ),
    .cfg_continuous_i ( buff_continuous ),
    .cfg_overflow_i   ( buff_ovflw_mode ),
    .cfg_flevel_i     ( buff_flevel     ),
    .cfg_en_i         ( buff_cfg_en     ),
    .cfg_clr_i        ( buff_cfg_clr    ),
    .cfg_smpls_left_o ( buff_bytes_left ),
    .cfg_en_o         ( buff_en         ),

    .wr_addr_o        ( buff_wr_addr    ),
    .rd_addr_o        ( buff_rd_addr    ),

    .wr_ready_i       ( buff_wr_ready_i ),

    .rd_ready_i       ( buff_rd_ready_i ),
    .rd_valid_o       ( buff_rd_valid_o ),

    .flevel_event_o   ( buff_event_o    )
  );


  pulp_clock_gating l2_side_cg_i
  (
    .clk_i,
    .en_i      ( ~intf_shtdwn_i ),
    .test_en_i ( test_mode_i    ),
    .clk_o     ( clk_l2_side    )
  );


  always_ff @(posedge clk_i, negedge rst_ni) begin
    if(~rst_ni) begin
      flag_valid_q <= 1'b0;
      flag_data_q  <= '0;
      flag_cnt_q   <= '0;
      flag_event_q <= 1'b0;
      flag_state_q <= IDLE;
    end
    else begin
      flag_valid_q <= flag_valid_n;
      flag_event_q <= flag_event_n;
      flag_state_q <= flag_state_n;

      if (flag_tf_valid | flag_event_clr)
        flag_cnt_q   <= flag_cnt_n;

      if (flag_valid_n)
        flag_data_q  <= flag_data_n;
    end
  end

endmodule
