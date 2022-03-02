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

// Authors: Yasmine Bennani <ybennani@student.ethz.ch>, Florian Glaser <glaserf@iis.ee.ethz.ch>

// register map
`define REG_L2_SADDR    3'b000 //BASEADDR+0x00
`define REG_L2_SIZE     3'b001 //BASEADDR+0x04
`define REG_L2_CFG      3'b010 //BASEADDR+0x08
`define REG_L2_UNUSED0  3'b011 //BASEADDR+0x0C
`define REG_L2_CURRADDR 3'b100 //BASEADDR+0x10
`define REG_L2_BYTELEFT 3'b101 //BASEADDR+0x14

`define REG_BUF_SADDR   4'b0000 //GEN_BASEADDR+0x00
`define REG_BUF_SIZE    4'b0001 //GEN_BASEADDR+0x04
`define REG_BUF_FLEVEL  4'b0010 //GEN_BASEADDR+0x08
`define REG_BUF_CFG     4'b0011 //GEN_BASEADDR+0x0C
`define REG_BUF_WRPTR   4'b0100 //GEN_BASEADDR+0x10
`define REG_BUF_RDPTR   4'b0101 //GEN_BASEADDR+0x14
`define REG_BUF_SMPLEFT 4'b0110 //GEN_BASEADDR+0x18

`define REG_AFE_CH_MASK 4'b1000 //GEN_BASEADDR+0x20
`define REG_AFE_CH_MODE 4'b1001 //GEN_BASEADDR+0x24
`define REG_MASK_MODE   4'b1010 //GEN_BASEADDR+0x28
`define REG_UNUSED0     4'b1011 //GEN_BASEADDR+0x2C
`define REG_FLAGS_CFG   4'b1100 //GEN_BASEADDR+0x30
`define REG_FLAGS_DATA  4'b1101 //GEN_BASEADDR+0x34
`define REG_FLAGS_CNT   4'b1110 //GEN_BASEADDR+0x38

module afe_reg_if #(
  parameter AFE_RX_TYPE       = 0,
  parameter AFE_NUM_CHS       = 8,
  parameter AFE_CHID_WIDTH    = 4,
  parameter AFE_SUBCHID_WIDTH = 2,
  parameter AFE_FLAG_WIDTH    = 4,
  parameter BUFF_AWIDTH       = 10,
  parameter BUFF_TRANS_SIZE   = 10,
  parameter L2_AWIDTH_NOAL    = 12,
  parameter L2_TRANS_SIZE     = 16,
  parameter L2_NUM_CHS        = 8,
  parameter FLAG_CNT_WIDTH    = 8
) (
  input  logic                       clk_i,
  input  logic                       rst_ni,
  input  logic                       test_mode_i,

  input  logic                [31:0] cfg_wdata_i,
  input  logic                [10:0] cfg_addr_i,
  input  logic                       cfg_valid_i,
  input  logic                       cfg_rwn_i,
  output logic                [31:0] cfg_rdata_o,
  output logic                       cfg_ready_o,

  output logic                       cfg_flag_en_o,
  output logic  [AFE_FLAG_WIDTH-1:0] cfg_flag_mask_o,
  input  logic                [31:0] cfg_flag_data_i,
  input  logic  [FLAG_CNT_WIDTH-1:0] cfg_flag_cnt_i,
  output logic                       cfg_flag_clr_o,

  // buffer configuration signals
  output logic     [BUFF_AWIDTH-1:0] cfg_buff_startaddr_o,
  output logic [BUFF_TRANS_SIZE-1:0] cfg_buff_size_o,
  output logic                       cfg_buff_continuous_o,
  output logic                       cfg_buff_overflow_o,
  output logic                       cfg_buff_en_o,
  output logic                       cfg_buff_clr_o,
  output logic [BUFF_TRANS_SIZE-1:0] cfg_buff_flevel_o,
  input  logic                       cfg_buff_en_i,
  input  logic     [BUFF_AWIDTH-1:0] cfg_buff_curr_waddr_i,
  input  logic     [BUFF_AWIDTH-1:0] cfg_buff_curr_raddr_i,
  input  logic [BUFF_TRANS_SIZE-1:0] cfg_buff_bytes_left_i,

  output logic     [AFE_NUM_CHS-1:0] cfg_afe_ch_mask_o,
  output logic                       cfg_afe_ch_en_mode_o,
  output logic  [AFE_CHID_WIDTH-1:0] cfg_afe_ch_en_chid_o,

  output logic                 [1:0] cfg_data_mask_mode_o,

  // L2 channel configuration signals
  output logic [L2_NUM_CHS-1:0]   [L2_AWIDTH_NOAL-1:0] cfg_l2_startaddr_o,
  output logic [L2_NUM_CHS-1:0]    [L2_TRANS_SIZE-1:0] cfg_l2_size_o,
  output logic [L2_NUM_CHS-1:0]                  [1:0] cfg_l2_datasize_o,
  output logic [L2_NUM_CHS-1:0]                        cfg_l2_continuous_o,
  output logic [L2_NUM_CHS-1:0]                        cfg_l2_en_o,
  output logic [L2_NUM_CHS-1:0]                        cfg_l2_clr_o,
  output logic [L2_NUM_CHS-1:0][AFE_SUBCHID_WIDTH-1:0] cfg_l2_subchid_o,
  output logic [L2_NUM_CHS-1:0]   [AFE_CHID_WIDTH-1:0] cfg_l2_chid_o,
  input  logic [L2_NUM_CHS-1:0]                        cfg_l2_en_i,
  input  logic [L2_NUM_CHS-1:0]   [L2_AWIDTH_NOAL-1:0] cfg_l2_curr_addr_i,
  input  logic [L2_NUM_CHS-1:0]    [L2_TRANS_SIZE-1:0] cfg_l2_bytes_left_i
);

  /* buffer registers */
  logic     [BUFF_AWIDTH-1:0] r_buff_startaddr;
  logic [BUFF_TRANS_SIZE-1:0] r_buff_size;
  logic                       r_buff_continuous;
  logic                       r_buff_overflow;
  logic                       r_buff_en;
  logic                       r_buff_clr;
  logic [BUFF_TRANS_SIZE-1:0] r_buff_flevel;

  /* misc and flag configuration */
  logic     [AFE_NUM_CHS-1:0] r_afe_ch_mask;
  logic                       r_afe_ch_en_mode;
  logic  [AFE_CHID_WIDTH-1:0] r_afe_ch_en_chid;

  logic                 [1:0] r_data_mask_mode;
  logic                       r_flag_en;
  logic  [AFE_FLAG_WIDTH-1:0] r_flag_mask;

  /* L2 channel registers */
  logic [L2_NUM_CHS-1:0]   [L2_AWIDTH_NOAL-1:0] r_l2_startaddr;
  logic [L2_NUM_CHS-1:0]    [L2_TRANS_SIZE-1:0] r_l2_size;
  logic [L2_NUM_CHS-1:0]                  [1:0] r_l2_datasize;
  logic [L2_NUM_CHS-1:0]                        r_l2_continuous;
  logic [L2_NUM_CHS-1:0]                        r_l2_en;
  logic [L2_NUM_CHS-1:0]                        r_l2_clr;
  logic [L2_NUM_CHS-1:0][AFE_SUBCHID_WIDTH-1:0] r_l2_subchid;
  logic [L2_NUM_CHS-1:0]   [AFE_CHID_WIDTH-1:0] r_l2_chid;

  /* regular signals */
  logic  [2:0] s_reg_sel_w, s_reg_sel_r;
  logic  [4:0] s_ch_sel_w, s_ch_sel_r;


  assign s_reg_sel_w = (cfg_valid_i & ~cfg_rwn_i) ? cfg_addr_i[2:0] : 'h0;
  assign s_reg_sel_r = (cfg_valid_i &  cfg_rwn_i) ? cfg_addr_i[2:0] : 'h0;

  assign s_ch_sel_w  = (cfg_valid_i & ~cfg_rwn_i) ? cfg_addr_i[7:3] : 'h0;
  assign s_ch_sel_r  = (cfg_valid_i &  cfg_rwn_i) ? cfg_addr_i[7:3] : 'h0;

  assign cfg_l2_startaddr_o    = r_l2_startaddr;
  assign cfg_l2_size_o         = r_l2_size;
  assign cfg_l2_datasize_o     = r_l2_datasize;
  assign cfg_l2_continuous_o   = r_l2_continuous;
  assign cfg_l2_en_o           = r_l2_en;
  assign cfg_l2_clr_o          = r_l2_clr;
  assign cfg_l2_subchid_o      = ((AFE_RX_TYPE == 1) || (AFE_RX_TYPE == 3)) ? r_l2_subchid : '0;
  assign cfg_l2_chid_o         = ((AFE_RX_TYPE == 2) || (AFE_RX_TYPE == 3)) ? r_l2_chid    : '0;

  assign cfg_buff_startaddr_o  = r_buff_startaddr;
  assign cfg_buff_size_o       = r_buff_size;
  assign cfg_buff_flevel_o     = r_buff_flevel;
  assign cfg_buff_en_o         = r_buff_en;
  assign cfg_buff_clr_o        = r_buff_clr;
  assign cfg_buff_continuous_o = r_buff_continuous;
  assign cfg_buff_overflow_o   = r_buff_overflow;

  assign cfg_afe_ch_mask_o     = r_afe_ch_mask;
  assign cfg_afe_ch_en_mode_o  = r_afe_ch_en_mode;
  assign cfg_afe_ch_en_chid_o  = r_afe_ch_en_chid;

  assign cfg_data_mask_mode_o  = r_data_mask_mode;

  assign cfg_flag_en_o         = r_flag_en;
  assign cfg_flag_mask_o       = r_flag_mask;


  /* write process */
  always_ff @(posedge clk_i, negedge rst_ni) begin
    if(~rst_ni) begin
      r_l2_startaddr    <=  '0;
      r_l2_size         <=  '0;
      r_l2_datasize     <=  {L2_NUM_CHS{2'b10}};
      r_l2_continuous   <=  '0;
      r_l2_en           <=  '0;
      r_l2_clr          <=  '0;
      r_l2_subchid      <=  '0;
      r_l2_chid         <=  '0;

      r_buff_startaddr  <=  '0;
      r_buff_size       <=  '0;
      r_buff_continuous <= 1'b0;
      r_buff_overflow   <= 1'b0;
      r_buff_en         <= 1'b0;
      r_buff_clr        <= 1'b0;
      r_buff_flevel     <=  '0;

      r_afe_ch_mask     <=  '1;
      r_afe_ch_en_mode  <= 1'b0;
      r_afe_ch_en_chid  <=  '0;

      r_data_mask_mode  <=  '0;
      r_flag_en         <= 1'b0;
      r_flag_mask       <=  '0;
    end 
    else begin
      r_l2_en    <= '0;
      r_l2_clr   <= '0;

      r_buff_en  <= 1'b0;
      r_buff_clr <= 1'b0;

      if (cfg_valid_i & ~cfg_rwn_i) begin
        // address space at top for generic and buffer config
        if (s_ch_sel_w[4:1] == 4'b1111) begin
          case ({s_ch_sel_w[0],s_reg_sel_w})
            // buffer registers
            `REG_BUF_SADDR:
              r_buff_startaddr  <= cfg_wdata_i[BUFF_AWIDTH-1:0];
            `REG_BUF_SIZE:
              r_buff_size       <= cfg_wdata_i[BUFF_TRANS_SIZE-1:0];
            `REG_BUF_FLEVEL:
              r_buff_flevel     <= cfg_wdata_i[BUFF_TRANS_SIZE-1:0];
            `REG_BUF_CFG: begin
              r_buff_clr        <= cfg_wdata_i[5];
              r_buff_en         <= cfg_wdata_i[4];
              r_buff_overflow   <= cfg_wdata_i[3];
              r_buff_continuous <= cfg_wdata_i[0];
            end
            `REG_AFE_CH_MASK:
              r_afe_ch_mask     <= cfg_wdata_i[AFE_NUM_CHS-1:0];
            `REG_AFE_CH_MODE: begin
              r_afe_ch_en_chid  <= cfg_wdata_i[AFE_CHID_WIDTH-1:0];
              r_afe_ch_en_mode  <= cfg_wdata_i[31];
            end
            `REG_MASK_MODE:
              r_data_mask_mode  <= cfg_wdata_i[1:0];
            `REG_FLAGS_CFG: begin
              r_flag_en         <= cfg_wdata_i[0];
              r_flag_mask       <= cfg_wdata_i[8 +: AFE_FLAG_WIDTH];
            end
          endcase
        end
        else begin
          case (s_reg_sel_w)
            // L2 channel registers
            `REG_L2_SADDR:
              r_l2_startaddr[s_ch_sel_w]   <= {cfg_wdata_i[L2_AWIDTH_NOAL-1:2],2'b00};
            `REG_L2_SIZE:
              r_l2_size[s_ch_sel_w]        <= {cfg_wdata_i[L2_TRANS_SIZE-1:2],2'b00};
            `REG_L2_CFG: begin
              r_l2_clr[s_ch_sel_w]         <= cfg_wdata_i[5];
              r_l2_en[s_ch_sel_w]          <= cfg_wdata_i[4];
              r_l2_datasize[s_ch_sel_w]    <= cfg_wdata_i[2:1];
              r_l2_continuous[s_ch_sel_w]  <= cfg_wdata_i[0];
              if ((AFE_RX_TYPE == 1) || (AFE_RX_TYPE == 3))
                r_l2_subchid[s_ch_sel_w]   <= cfg_wdata_i[16 +: AFE_SUBCHID_WIDTH];
              if ((AFE_RX_TYPE == 2) || (AFE_RX_TYPE == 3))
                r_l2_chid[s_ch_sel_w]      <= cfg_wdata_i[24 +: AFE_CHID_WIDTH];
            end
          endcase
        end

      end
    end
  end 

  /* read decoder */
  always_comb begin
    cfg_rdata_o    = '0;
    cfg_flag_clr_o = 1'b0;

    // address space at top for generic and buffer config
    if (s_ch_sel_r[4:1] == 4'b1111) begin
      case ({s_ch_sel_r[0],s_reg_sel_r})
        // buffer registers
        `REG_BUF_SADDR:
          cfg_rdata_o[BUFF_AWIDTH-1:0]     = r_buff_startaddr;
        `REG_BUF_SIZE:
          cfg_rdata_o[BUFF_TRANS_SIZE-1:0] = r_buff_size;
        `REG_BUF_FLEVEL:
          cfg_rdata_o[BUFF_TRANS_SIZE-1:0] = r_buff_flevel;
        `REG_BUF_CFG:
          cfg_rdata_o = {27'h0,r_buff_en,r_buff_overflow,2'b10,r_buff_continuous};
        `REG_BUF_WRPTR:
          cfg_rdata_o[BUFF_AWIDTH-1:0]     = cfg_buff_curr_waddr_i;
        `REG_BUF_RDPTR:
          cfg_rdata_o[BUFF_AWIDTH-1:0]     = cfg_buff_curr_raddr_i;
        `REG_BUF_SMPLEFT:
          cfg_rdata_o[BUFF_TRANS_SIZE-1:0] = cfg_buff_bytes_left_i;

        `REG_AFE_CH_MASK:
          cfg_rdata_o[AFE_NUM_CHS-1:0]     = r_afe_ch_mask;
        `REG_AFE_CH_MODE:
          cfg_rdata_o = {r_afe_ch_en_mode,{(31-AFE_CHID_WIDTH){1'b0}},r_afe_ch_en_chid};
        `REG_MASK_MODE:
          cfg_rdata_o = {30'h0,r_data_mask_mode};
        `REG_FLAGS_CFG:
          cfg_rdata_o = {{(24-AFE_FLAG_WIDTH){1'b0}},r_flag_mask,7'h0,r_flag_en};
        `REG_FLAGS_DATA:
          cfg_rdata_o = cfg_flag_data_i;
        `REG_FLAGS_CNT: begin
          cfg_rdata_o[FLAG_CNT_WIDTH-1:0]  = cfg_flag_cnt_i;
          cfg_flag_clr_o = 1'b1;
        end

        default:
          cfg_rdata_o = '0;
      endcase

    end
    else begin
      case (s_reg_sel_r)
        // L2 channel registers
        `REG_L2_SADDR:
          cfg_rdata_o[L2_AWIDTH_NOAL-1:0] = r_l2_startaddr[s_ch_sel_r];
        `REG_L2_SIZE:
          cfg_rdata_o[L2_TRANS_SIZE-1:0]  = r_l2_size[s_ch_sel_r];
        `REG_L2_CFG: begin
          cfg_rdata_o[24 +:    AFE_CHID_WIDTH] = r_l2_chid[s_ch_sel_r];
          cfg_rdata_o[16 +: AFE_SUBCHID_WIDTH] = r_l2_subchid[s_ch_sel_r];

          cfg_rdata_o[4]   = cfg_l2_en_i[s_ch_sel_r];
          cfg_rdata_o[2:1] = r_l2_datasize[s_ch_sel_r];
          cfg_rdata_o[0]   = r_l2_continuous[s_ch_sel_r];
        end
        `REG_L2_CURRADDR:
          cfg_rdata_o[L2_AWIDTH_NOAL-1:0] = cfg_l2_curr_addr_i[s_ch_sel_r];
        `REG_L2_BYTELEFT:
          cfg_rdata_o[L2_TRANS_SIZE-1:0]  = cfg_l2_bytes_left_i[s_ch_sel_r];

        default:
          cfg_rdata_o = '0;
      endcase
    end
  end

  assign cfg_ready_o = 1'b1;

endmodule 
