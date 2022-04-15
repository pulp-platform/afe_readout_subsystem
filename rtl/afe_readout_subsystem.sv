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

module afe_readout_subsystem
  import afe_parameters_pkg::*;
#(
  parameter APB_ADDR_WIDTH  = 32,
  parameter L2_AWIDTH_NOAL  = 12,
  parameter BUFF_AWIDTH     = 10,
  parameter BUFF_TRANS_SIZE = 16,
  parameter BUFF_BIST_EN    = 0,
  parameter AFE_DATA_WIDTH  = 32
)(
  input  logic     clk_i,
  input  logic     rst_ni,
  input  logic     test_mode_i,

  input  logic     buff_bist_en_i,
  output logic     buff_bist_done_o,
  output logic     buff_bist_fail_o,

  /* APB interface for configuration */
  input  logic                        apb_sel_i,
  input  logic                        apb_en_i,
  input  logic                        apb_write_i,
  input  logic   [APB_ADDR_WIDTH-1:0] apb_address_i,
  input  logic                 [31:0] apb_wdata_i,
  output logic                        apb_ready_o,
  output logic                        apb_slverr_o,
  output logic                 [31:0] apb_rdata_o,

  /* udma intf protocol signals */
  input  logic                        udma_aferx_ready_i,
  output logic                        udma_aferx_valid_o,
  output logic   [AFE_DATA_WIDTH-1:0] udma_aferx_data_o,
  output logic   [L2_AWIDTH_NOAL-1:0] udma_aferx_addr_o,
  output logic                  [1:0] udma_aferx_size_o,

  /* udma flag channels */
  input  logic [NUM_AFE_RX-1:0]       udma_flag_ready_i,
  output logic [NUM_AFE_RX-1:0]       udma_flag_valid_o,
  output logic [NUM_AFE_RX-1:0][31:0] udma_flag_data_o,

  /* events */
  output logic            [AFE_RX_NUM_L2CH_TOT-1:0] l2_event_o,
  output logic                     [NUM_AFE_RX-1:0] buff_event_o,
  output logic                     [NUM_AFE_RX-1:0] flag_event_o,

  /* AFE data and async valid */
  input  logic [NUM_AFE_RX-1:0]                     afe_data_valid_i,
  input  logic [NUM_AFE_RX-1:0][AFE_DATA_WIDTH-1:0] afe_data_i
);

  /* configuration */
  logic                 [10:0] cfg_addr;
  logic                 [31:0] cfg_wdata;
  logic [NUM_AFE_RX-1:0]       cfg_valid;
  logic [NUM_AFE_RX-1:0]       cfg_ready;
  logic                        cfg_rwn;
  logic [NUM_AFE_RX-1:0][31:0] cfg_rdata;
  
  logic [NUM_AFE_RX-1:0]       afe_top_clk;
  logic                        udma_shutdown;

  /* buffer */
  logic [AFE_DATA_WIDTH-1:0] buff_rdata;
  logic                      buff_cen, buff_wen;
  logic [AFE_DATA_WIDTH-1:0] buff_wdata;
  logic    [BUFF_AWIDTH-1:0] buff_addr;

  /* AFE top signals to be arbitrated */
  /* AFE input */
  logic [NUM_AFE_RX-1:0]                     afe_valid;
  logic [NUM_AFE_RX-1:0][AFE_DATA_WIDTH-1:0] afe_data;

  /* buffer rd+wr requests, addresses */
  logic [NUM_AFE_RX-1:0] buff_rvalid;
  logic [NUM_AFE_RX-1:0] buff_wr_ready;
  logic [NUM_AFE_RX-1:0] buff_rd_ready, buff_rd_valid;

  logic [NUM_AFE_RX-1:0]   [BUFF_AWIDTH-1:0] buff_waddr, buff_raddr;

  /* L2 write channel */
  logic [NUM_AFE_RX-1:0][L2_AWIDTH_NOAL-1:0] l2_addr;
  logic [NUM_AFE_RX-1:0][AFE_DATA_WIDTH-1:0] l2_wdata;
  logic [NUM_AFE_RX-1:0]               [1:0] l2_size;

  /* arbitrated signals to udma interface */
  logic                      intf_valid, intf_ready;
  logic [AFE_DATA_WIDTH-1:0] intf_wdata;
  logic [L2_AWIDTH_NOAL-1:0] intf_addr;
  logic                [1:0] intf_size;


  afe_ro_conf_if #(
    .NUM_AFE        ( NUM_AFE_RX     ),
    .APB_ADDR_WIDTH ( APB_ADDR_WIDTH )
  )
  afe_ro_conf_if_i (
    .clk_i,
    .rst_ni,
    .test_mode_i,

    .apb_sel_i,
    .apb_en_i,
    .apb_write_i,
    .apb_address_i,
    .apb_wdata_i,
    .apb_ready_o,
    .apb_slverr_o,
    .apb_rdata_o,

    .cfg_addr_o    ( cfg_addr      ),
    .cfg_wdata_o   ( cfg_wdata     ),
    .cfg_valid_o   ( cfg_valid     ),
    .cfg_ready_i   ( cfg_ready     ),
    .cfg_rwn_o     ( cfg_rwn       ),
    .cfg_rdata_i   ( cfg_rdata     ),

    .udma_shtdwn_o ( udma_shutdown ),
    .afe_top_clk_o ( afe_top_clk   )
  );


  for(genvar I=0; I<NUM_AFE_RX; I++) begin: afe_top_gen_loop
    afe_top #(
      .AFE_RX_TYPE       ( AFE_RX_TYPE[I]       ),

      .L2_AWIDTH_NOAL    ( L2_AWIDTH_NOAL       ),
      .L2_TRANS_SIZE     ( AFE_RX_TRANS_SIZE[I] ),
      .L2_NUM_CHS        ( AFE_RX_NUM_L2CHS[I]  ),

      .BUFF_AWIDTH       ( BUFF_AWIDTH          ),
      .BUFF_TRANS_SIZE   ( BUFF_TRANS_SIZE      ),

      .AFE_DATA_WIDTH    ( AFE_DATA_WIDTH       ),
      .AFE_NUM_CHS       ( AFE_RX_NUM_CHS[I]    ),
      .AFE_PL_WIDTH      ( AFE_PL_DATA_W[I]     ),
      .AFE_CHID_LSB      ( AFE_RX_CHID_LSB[I]   ),
      .AFE_CHID_WIDTH    ( AFE_RX_CHID_WIDTH[I] ),
      .AFE_SUBCHID_LSB   ( AFE_RX_SUBCHID_LSB[I]   ),
      .AFE_SUBCHID_WIDTH ( AFE_RX_SUBCHID_WIDTH[I] ),
      .AFE_FLAG_LSB      ( AFE_RX_FLAG_LSB[I]   ),
      .AFE_FLAG_WIDTH    ( AFE_RX_FLAG_WIDTH[I] ),

      .FEATURE_FLAG      ( AFE_FLAG_MASK[I]     )
    )
    afe_top_i (
      .clk_i             ( afe_top_clk[I]       ),
      .rst_ni,
      .test_mode_i,
  
      .cfg_addr_i        ( cfg_addr             ),
      .cfg_wdata_i       ( cfg_wdata            ),
      .cfg_valid_i       ( cfg_valid[I]         ),
      .cfg_rwn_i         ( cfg_rwn              ),
      .cfg_rdata_o       ( cfg_rdata[I]         ),
      .cfg_ready_o       ( cfg_ready[I]         ),

      .l2_ch_event_o     ( l2_event_o[AFE_RX_OFFS_L2CH[I] +: AFE_RX_NUM_L2CHS[I]] ),
      .buff_event_o      ( buff_event_o[I]      ),
      .flag_event_o      ( flag_event_o[I]      ),

      .buff_waddr_o      ( buff_waddr[I]        ),
      .buff_raddr_o      ( buff_raddr[I]        ),

      .buff_rvalid_i     ( buff_rvalid[I]       ),
      .buff_rdata_i      ( buff_rdata           ),

      .l2_addr_o         ( l2_addr[I]           ),
      .l2_size_o         ( l2_size[I]           ),
      .l2_wdata_o        ( l2_wdata[I]          ),

      .flag_valid_o      ( udma_flag_valid_o[I] ),
      .flag_ready_i      ( udma_flag_ready_i[I] ),
      .flag_data_o       ( udma_flag_data_o[I]  ),

      .buff_wr_ready_i   ( buff_wr_ready[I]     ),
      .buff_rd_valid_o   ( buff_rd_valid[I]     ),
      .buff_rd_ready_i   ( buff_rd_ready[I]     ),

      .afe_valid_o       ( afe_valid[I]         ),
      .afe_data_o        ( afe_data[I]          ),
  
      .afe_valid_async_i ( afe_data_valid_i[I]  ),
      .afe_data_async_i  ( afe_data_i[I]        )
    );
  end


  afe_ro_arbiter #(
    .NUM_AFE     ( NUM_AFE_RX     ),
    .DATA_WIDTH  ( AFE_DATA_WIDTH ),
    .BUFF_AWIDTH ( BUFF_AWIDTH    ),
    .L2_AWIDTH   ( L2_AWIDTH_NOAL )
  )
  afe_ro_arbiter_i (
    .clk_i,
    .rst_ni,
    .test_mode_i,

    .buff_cen_o     ( buff_cen      ),
    .buff_rwn_o     ( buff_wen      ),
    .buff_addr_o    ( buff_addr     ),
    .buff_wdata_o   ( buff_wdata    ),

    .buff_rvalid_o  ( buff_rvalid   ),

    .intf_valid_o   ( intf_valid    ),
    .intf_ready_i   ( intf_ready    ),
    .intf_wdata_o   ( intf_wdata    ),
    .intf_addr_o    ( intf_addr     ),
    .intf_size_o    ( intf_size     ),

    .afe_valid_i    ( afe_valid     ),
    .afe_data_i     ( afe_data      ),

    .wr_ready_o     ( buff_wr_ready ),
    .rd_ready_o     ( buff_rd_ready ),
    .rd_valid_i     ( buff_rd_valid ),

    .l2_wdata_i     ( l2_wdata      ),
    .l2_addr_i      ( l2_addr       ),
    .l2_size_i      ( l2_size       ),

    .buff_wr_addr_i ( buff_waddr    ),
    .buff_rd_addr_i ( buff_raddr    )
  );


  // Protocol interface adapter and transaction buffer to communicate with the uDMA
  afe_ro_udma_if #(
    .L2_DATA_WIDTH  ( 32             ),
    .L2_AWIDTH_NOAL ( L2_AWIDTH_NOAL )
  )
  afe_ro_udma_if_i (
    .clk_i,
    .rst_ni,
    .test_mode_i,

    .afero_valid_i   ( intf_valid         ),
    .afero_ready_o   ( intf_ready         ),
    .afero_buff_ce_o (                    ),
    .afero_wdata_i   ( intf_wdata         ),
    .afero_addr_i    ( intf_addr          ),
    .afero_size_i    ( intf_size          ),

    .udma_shtdwn_i   ( udma_shutdown      ),
    .udma_ready_i    ( udma_aferx_ready_i ),
    .udma_valid_o    ( udma_aferx_valid_o ),
    .udma_data_o     ( udma_aferx_data_o  ),
    .udma_addr_o     ( udma_aferx_addr_o  ),
    .udma_size_o     ( udma_aferx_size_o  )
  );


  afe_ro_sram_buffer #(
    .AFE_DATA_WIDTH ( AFE_DATA_WIDTH ),
    .ADDR_WIDTH     ( BUFF_AWIDTH    ),
    .BIST_EN        ( BUFF_BIST_EN   )
  ) buffer_i (
    .clk_i,
    .rst_ni,

    .bist_en_i   ( buff_bist_en_i   ),
    .bist_done_o ( buff_bist_done_o ),
    .bist_fail_o ( buff_bist_fail_o ),

    .cen_i       ( buff_cen         ),
    .wen_i       ( buff_wen         ),
    .data_i      ( buff_wdata       ),
    .addr_i      ( buff_addr        ),
    .data_o      ( buff_rdata       )
  );

endmodule
