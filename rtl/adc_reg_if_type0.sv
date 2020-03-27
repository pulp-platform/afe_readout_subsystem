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
`define REG_RX_SADDR    3'b000 //BASEADDR+0x00
`define REG_RX_SIZE     3'b001 //BASEADDR+0x04
`define REG_RX_CFG      3'b010 //BASEADDR+0x08

`define REG_BUF_SADDR   4'b0000 //GEN_BASEADDR+0x00
`define REG_BUF_SIZE    4'b0001 //GEN_BASEADDR+0x04
`define REG_BUF_FLEVEL  4'b0010 //GEN_BASEADDR+0x08
`define REG_BUF_CFG     4'b0011 //GEN_BASEADDR+0x0C

`define REG_CH_MASK     4'b1000 //GEN_BASEADDR+0x20
`define REG_BUF_MODE    4'b1001 //GEN_BASEADDR+0x24
`define REG_MASK_MODE   4'b1010 //GEN_BASEADDR+0x28
`define REG_UNUSED0     4'b1011 //GEN_BASEADDR+0x2C
`define REG_FLAGS_EN    4'b1100 //GEN_BASEADDR+0x30
`define REG_FLAGS_DATA  4'b1101 //GEN_BASEADDR+0x34

module adc_reg_if_type0 #(
  parameter L2_AWIDTH_NOAL  = 12,
  parameter UDMA_TRANS_SIZE = 16,
  parameter TRANS_SIZE      = 16,
  parameter ADC_NUM_CHS     = 8,
  parameter ADC_CHID_WIDTH  = 4,
  parameter L2_NUM_CHS      = ADC_NUM_CHS,
  parameter BUF_AWIDTH      = 10,
  parameter BUF_TRANS_SIZE  = 10
) (
  input  logic                      clk_i,
  input  logic                      rstn_i,

  input  logic               [31:0] cfg_data_i,
  input  logic               [10:0] cfg_addr_i,
  input  logic                      cfg_valid_i,
  input  logic                      cfg_rwn_i,
  output logic               [31:0] cfg_data_o,
  output logic                      cfg_ready_o,

  output logic                                        cfg_flag_event_en_o,
  input  logic                                        cfg_flag_event_i,
  input  logic                                 [31:0] cfg_flag_data_i,

  // RX registers configuration signals
  output logic  [L2_NUM_CHS-1:0] [L2_AWIDTH_NOAL-1:0] cfg_rx_startaddr_o,
  output logic  [L2_NUM_CHS-1:0][UDMA_TRANS_SIZE-1:0] cfg_rx_size_o,
  output logic  [L2_NUM_CHS-1:0]                      cfg_rx_continuous_o,
  output logic  [L2_NUM_CHS-1:0]                      cfg_rx_en_o,
  output logic  [L2_NUM_CHS-1:0]                      cfg_rx_clr_o,
  input  logic  [L2_NUM_CHS-1:0]                      cfg_rx_en_i,
  input  logic  [L2_NUM_CHS-1:0] [L2_AWIDTH_NOAL-1:0] cfg_rx_curr_addr_i,
  input  logic  [L2_NUM_CHS-1:0][UDMA_TRANS_SIZE-1:0] cfg_rx_bytes_left_i,

  // BUF registers configuration signals
  output logic                       [BUF_AWIDTH-1:0] cfg_buf_startaddr_o,
  output logic                   [BUF_TRANS_SIZE-1:0] cfg_buf_size_o,
  output logic                   [BUF_TRANS_SIZE-1:0] cfg_buf_flevel_o,
  output logic                                        cfg_buf_continuous_o,
  output logic                                        cfg_buf_en_o,
  output logic                                        cfg_buf_clr_o,
  input  logic                                        cfg_buf_en_i,
  input  logic                       [BUF_AWIDTH-1:0] cfg_buf_curr_addr_i,
  input  logic                   [BUF_TRANS_SIZE-1:0] cfg_buf_bytes_left_i,

  output logic                                  [1:0] cfg_data_mask_mode_o,

  output logic                      [ADC_NUM_CHS-1:0] cfg_buf_ch_mask_o,
  output logic                                        cfg_buf_en_mode_o,
  output logic                   [ADC_CHID_WIDTH-1:0] cfg_buf_en_chid_o
);

  logic  [L2_NUM_CHS-1:0][L2_AWIDTH_NOAL-1:0] r_rx_startaddr;
  logic  [L2_NUM_CHS-1:0]    [TRANS_SIZE-1:0] r_rx_size;
  logic  [L2_NUM_CHS-1:0]                     r_rx_continuous;
  logic  [L2_NUM_CHS-1:0]                     r_rx_en;
  logic  [L2_NUM_CHS-1:0]                     r_rx_clr;

  logic                      [BUF_AWIDTH-1:0] r_buf_startaddr;
  logic                  [BUF_TRANS_SIZE-1:0] r_buf_size;
  logic                     [ADC_NUM_CHS-1:0] r_ch_mask;
  logic                                 [1:0] r_data_mask_mode;
  logic                                       r_buf_en_mode;
  logic                  [ADC_CHID_WIDTH-1:0] r_buf_en_chid;
  logic                  [BUF_TRANS_SIZE-1:0] r_buf_flevel;
  logic                                       r_buf_en;
  logic                                       r_buf_clr;
  logic                                       r_buf_continuous;

  logic                                       r_flag_evt_en;
  logic                                [31:0] r_flag_data;

  logic  [2:0] s_reg_sel_w, s_reg_sel_r;
  logic  [4:0] s_ch_sel_w, s_ch_sel_r;


  assign s_reg_sel_w = (cfg_valid_i & ~cfg_rwn_i) ? cfg_addr_i[2:0] : 'h0;
  assign s_reg_sel_r = (cfg_valid_i &  cfg_rwn_i) ? cfg_addr_i[2:0] : 'h0;

  assign s_ch_sel_w  = (cfg_valid_i & ~cfg_rwn_i) ? cfg_addr_i[7:3] : 'h0;
  assign s_ch_sel_r  = (cfg_valid_i &  cfg_rwn_i) ? cfg_addr_i[7:3] : 'h0;

  assign cfg_rx_startaddr_o   = r_rx_startaddr;
  assign cfg_rx_continuous_o  = r_rx_continuous;
  assign cfg_rx_en_o          = r_rx_en;
  assign cfg_rx_clr_o         = r_rx_clr;

  assign cfg_buf_startaddr_o  = r_buf_startaddr;
  assign cfg_buf_size_o       = r_buf_size;
  assign cfg_buf_flevel_o     = r_buf_flevel;
  assign cfg_buf_en_o         = r_buf_en;
  assign cfg_buf_clr_o        = r_buf_clr;
  assign cfg_buf_continuous_o = r_buf_continuous;

  assign cfg_buf_ch_mask_o    = r_ch_mask;
  assign cfg_buf_en_mode_o    = r_buf_en_mode;
  assign cfg_buf_en_chid_o    = r_buf_en_chid;

  assign cfg_data_mask_mode_o = r_data_mask_mode;

  assign cfg_flag_event_en_o  = r_flag_evt_en;

  generate
    for (genvar I=0; I<L2_NUM_CHS; I++) begin
      assign cfg_rx_size_o[I][TRANS_SIZE-1:0] = r_rx_size[I];
      if (UDMA_TRANS_SIZE > TRANS_SIZE)
        assign cfg_rx_size_o[I][UDMA_TRANS_SIZE-1:TRANS_SIZE] = '0;
    end
  endgenerate

  always_ff @(posedge clk_i, negedge rstn_i) begin
    if(~rstn_i) begin
      r_rx_startaddr   <=  '0;
      r_rx_size        <=  '0;
      r_rx_continuous  <=  '0;
      r_rx_en          <=  '0;
      r_rx_clr         <=  '0;

      r_buf_startaddr  <=  '0;
      r_buf_size       <=  '0;
      r_ch_mask        <=  '1;
      r_buf_en_mode    <= 1'b0;
      r_buf_en_chid    <=  '0;
      r_data_mask_mode <=  '0;
      r_buf_flevel     <=  '0;
      r_buf_en         <= 1'b0;
      r_buf_clr        <= 1'b0;
      r_buf_continuous <= 1'b0;

      r_flag_evt_en    <= 1'b0;
      r_flag_data      <=  '0;
    end 
    else begin
      r_rx_en   <=  'h0;
      r_rx_clr  <=  'h0;

      r_buf_en  <=  1'b0;
      r_buf_clr <=  1'b0;

      if (cfg_flag_event_i)
        r_flag_data <= cfg_flag_data_i;
      else if ((cfg_valid_i & cfg_rwn_i) && (s_ch_sel_r[4:1] == 4'b1111) && ({s_ch_sel_r[0],s_reg_sel_r} == `REG_FLAGS_DATA))
        r_flag_data <= '0;

      if (cfg_valid_i & ~cfg_rwn_i) begin
        // address space at top for generic and buffer config
        if (s_ch_sel_w[4:1] == 4'b1111) begin
          case ({s_ch_sel_w[0],s_reg_sel_w})
            // BUFFER REGISTERS
            `REG_BUF_SADDR:
              r_buf_startaddr   <= cfg_data_i[BUF_AWIDTH-1:0];
            `REG_BUF_SIZE:
              r_buf_size        <= cfg_data_i[BUF_TRANS_SIZE-1:0];
            `REG_BUF_FLEVEL:
              r_buf_flevel      <= cfg_data_i[BUF_TRANS_SIZE-1:0];
            `REG_BUF_CFG: begin
              r_buf_clr         <= cfg_data_i[5];
              r_buf_en          <= cfg_data_i[4];
              r_buf_continuous  <= cfg_data_i[0];
            end
            `REG_CH_MASK:
              r_ch_mask         <= cfg_data_i[ADC_NUM_CHS-1:0];
            `REG_BUF_MODE: begin
              r_buf_en_chid     <= cfg_data_i[ADC_CHID_WIDTH-1:0];
              r_buf_en_mode     <= cfg_data_i[31];
            end
            `REG_MASK_MODE:
              r_data_mask_mode  <= cfg_data_i[1:0];
            `REG_FLAGS_EN:
              r_flag_evt_en     <= cfg_data_i[0];
          endcase
        end
        else begin
          case (s_reg_sel_w)
            // L2 REGISTERS
            `REG_RX_SADDR:
              r_rx_startaddr[s_ch_sel_w]   <= {cfg_data_i[L2_AWIDTH_NOAL-1:2],2'b00};
            `REG_RX_SIZE:
              r_rx_size[s_ch_sel_w]        <= {cfg_data_i[TRANS_SIZE-1:2],2'b00};
            `REG_RX_CFG: begin
              r_rx_clr[s_ch_sel_w]         <= cfg_data_i[5];
              r_rx_en[s_ch_sel_w]          <= cfg_data_i[4];
              r_rx_continuous[s_ch_sel_w]  <= cfg_data_i[0];
            end
          endcase
        end

      end
    end
  end 

  always_comb begin
    cfg_data_o = '0;

    // address space at top for generic and buffer config
    if (s_ch_sel_r[4:1] == 4'b1111) begin
      case ({s_ch_sel_r[0],s_reg_sel_r})
        // BUFFER REGISTERS
        `REG_BUF_SADDR:
          cfg_data_o[BUF_AWIDTH-1:0]      = cfg_buf_curr_addr_i;
        `REG_BUF_SIZE:
          cfg_data_o[BUF_TRANS_SIZE-1:0]  = cfg_buf_bytes_left_i;
        `REG_BUF_FLEVEL:
          cfg_data_o[BUF_TRANS_SIZE-1:0]  = r_buf_flevel;
        `REG_BUF_CFG:
          cfg_data_o = {27'h0,r_buf_en,1'b0,2'b10,r_buf_continuous};
        `REG_CH_MASK:
          cfg_data_o[ADC_NUM_CHS-1:0]     = r_ch_mask;
        `REG_BUF_MODE:
          cfg_data_o = {r_buf_en_mode,{(31-ADC_CHID_WIDTH){1'b0}},r_buf_en_chid};
        `REG_MASK_MODE:
          cfg_data_o = {30'h0,r_data_mask_mode};
        `REG_FLAGS_EN:
          cfg_data_o = {31'h0,r_flag_evt_en};
        `REG_FLAGS_DATA:
          cfg_data_o = r_flag_data;

        default:
          cfg_data_o = '0;
      endcase

    end
    else begin
      case (s_reg_sel_r)
        // L2 REGISTERS
        `REG_RX_SADDR:
          cfg_data_o[L2_AWIDTH_NOAL-1:0]  = cfg_rx_curr_addr_i[s_ch_sel_r];
        `REG_RX_SIZE:
          cfg_data_o[UDMA_TRANS_SIZE-1:0] = cfg_rx_bytes_left_i[s_ch_sel_r];
        `REG_RX_CFG:
          cfg_data_o = {27'h0,cfg_rx_en_i[s_ch_sel_r],1'b0,2'b10,r_rx_continuous[s_ch_sel_r]};

        default:
          cfg_data_o = '0;
      endcase
    end
  end

  assign cfg_ready_o = 1'b1;

endmodule 

