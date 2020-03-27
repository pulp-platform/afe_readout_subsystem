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

import afe_parameters_pkg::*;

module afe_readout_subsystem #(
  parameter APB_ADDR_WIDTH  = 32,
  parameter L2_AWIDTH_NOAL  = 12,
  parameter UDMA_TRANS_SIZE = 16, 
  parameter BUF_AWIDTH      = 10,
  parameter BUF_TRANS_SIZE  = 16,
  parameter ADC_DATA_WIDTH  = 32
)(
  input  logic     clk_i,
  input  logic     rstn_i,

  /* APB interface for configuration */
  input  logic                      apb_sel_i,
  input  logic                      apb_en_i,
  input  logic                      apb_write_i,
  input  logic [APB_ADDR_WIDTH-1:0] apb_address_i,
  input  logic               [31:0] apb_wdata_i,
  output logic                      apb_ready_o,
  output logic                      apb_slverr_o,
  output logic               [31:0] apb_rdata_o,

  /* UDMA IF protocol signals */
  input  logic                      udma_ready_i,
  output logic                      udma_valid_o,
  output logic [ADC_DATA_WIDTH-1:0] udma_data_o,
  output logic [L2_AWIDTH_NOAL-1:0] udma_addr_o,

  /* event signals */
  output logic            [AFE_RX_NUM_L2CH_TOT-1:0] l2_half_event_o,
  output logic                     [NUM_AFE_RX-1:0] flag_event_o,
  output logic                     [NUM_AFE_RX-1:0] buf_event_o,

  /* AFE signals */
  input  logic [NUM_AFE_RX-1:0]                     afe_data_valid_i,
  input  logic [NUM_AFE_RX-1:0][ADC_DATA_WIDTH-1:0] afe_data_i
);

  localparam LOG_N_ADC              = $clog2(NUM_AFE_RX);
  localparam LOG_MAX_CH_PER_ADC     = $clog2(AFE_RX_MAX_NUM_CH);
  localparam LOG_MAX_SUBCH_PER_ADC  = $clog2(AFE_RX_MAX_NUM_SUBCH);

  logic [NUM_AFE_RX-1:0]      [ADC_DATA_WIDTH-1:0] adc_rx_data_sync;
  logic [NUM_AFE_RX-1:0]                           adc_rx_valid_sync;
  logic [NUM_AFE_RX-1:0]                           adc_wr_req;
  logic [NUM_AFE_RX-1:0]                           r_adc_wr_req;                              // ADC requests to access the buffer in WRITE mode
  logic [NUM_AFE_RX-1:0]                           adc_rd_req;                                // ADC requests to access the buffer in READ  mode

  // configuration signals
  logic                                            cfg_valid;
  logic                                     [31:0] cfg_data_in;
  logic                                     [10:0] cfg_addr_in;
  logic [NUM_AFE_RX-1:0]                           cfg_valid_in;
  logic                                            cfg_rwn_in;
  logic [NUM_AFE_RX-1:0]                    [31:0] cfg_data_out;
  logic [NUM_AFE_RX-1:0]                           cfg_ready_out;
  logic                                      [2:0] s_cfg_adc_sel;

  logic                                            cfg_valid_shared;
  logic                                            cfg_ready_shared;
  logic                                     [31:0] cfg_data_shared;

  logic [NUM_AFE_RX-1:0]                     [1:0] data_mask_mode;

  logic [NUM_AFE_RX-1:0]                           adc_wr_grants;                             // Grant signal (grants one ADC to access the buffer)
  logic [NUM_AFE_RX-1:0]                           adc_rd_grants;                             // Grant signal (grants one ADC to store in L2) 
  logic  [LOG_N_ADC-1:0]                           adc_log_wr_grant;   
  logic  [LOG_N_ADC-1:0]                           adc_log_rd_grant;

  logic [NUM_AFE_RX-1:0]                           wr_grant_ack;  
  logic                                            rd_sgrant_ack;                             // Grant ack on one single bit 
  logic                                            wr_sgrant_ack;                             // Grant ack on one single bit -- 1 if at least one bit of wr_grant_ack is set
  logic                                            rd_anyGrant;
  logic                                            wr_anyGrant;

  logic [NUM_AFE_RX-1:0][AFE_RX_MAX_NUM_CH-1:0][L2_AWIDTH_NOAL-1:0] rx_addr;                  // Addresses coming out from the ADC top interfaces
  logic                                        [L2_AWIDTH_NOAL-1:0] rx_addr_sel;              // Address at which to write in L2 memory
  logic [NUM_AFE_RX-1:0]                           [BUF_AWIDTH-1:0] buf_addr;                 // Addresses coming out from the ADC top interfaces
  logic                                            [BUF_AWIDTH-1:0] buf_addr_sel;             // Address either to write or to read from in the buffer
  logic                                                             buf_rwn;                  // Buffer mode: 1 = READ MODE, 0 = WRITE MODE
  logic                                                             r_buf_rwn;
  logic [NUM_AFE_RX-1:0]                                            buf_grant;
  logic  [LOG_N_ADC-1:0]                                            buf_log_grant;            // ADC from read arbiter should be granted access to buffer in READ mode, otherwise ADC from write arbiter
  logic  [LOG_N_ADC-1:0]                                            r_buf_log_grant;

  logic [NUM_AFE_RX-1:0][AFE_RX_MAX_NUM_CH-1:0][AFE_RX_STRIDE_SIZE-1:0] rx_stride_size;

  logic [NUM_AFE_RX-1:0]                    [AFE_RX_MAX_FLAG_WIDTH-1:0] adc_flag_data;
  logic [NUM_AFE_RX-1:0]                                                adc_flag_data_vld;

  logic                                 [LOG_MAX_CH_PER_ADC-1:0] ch_id;                       // Channel ID (max 32 channels per ADC)
  logic                              [LOG_MAX_SUBCH_PER_ADC-1:0] subch_id;

  /* Buffer SRAM signals */
  logic                                     [ADC_DATA_WIDTH-1:0] buf_rdata;                   // Data coming out of the buffer
  logic                                     [ADC_DATA_WIDTH-1:0] buf_rdata_mask;
  logic                                     [ADC_DATA_WIDTH-1:0] buf_wdata;                   // If in write mode, data to write in the buffer
  logic                                                          buf_wen;                     // Buffer write enable -- active low
  logic                                                          buf_cen;                     // Buffer chip  enable -- active low

  /* UDMA IF Protocol internal signals */
  logic                                         udma_vtransfer_adc;                           // A valid transfer has occured on the UDMA side
  logic [NUM_AFE_RX-1:0][AFE_RX_MAX_NUM_CH-1:0] udma_vtransfer;                               // Duplicated version of udma_vtransfer_adc for the right adc and the right channel  
  logic [NUM_AFE_RX-1:0]                        udma_read_valid;
  logic [NUM_AFE_RX-1:0]                        r_udma_read_valid;                          
  logic                                         udma_valid;                 
  logic                                         udma_ce;                                      // Buffer chip enable set by the uDMA interface protocol if not in write mode
  logic                                         udma_shtdwn;

  integer CH_ID_LSB;
  integer CH_ID_WIDTH;
  integer SUBCH_ID_LSB;
  integer SUBCH_ID_WIDTH;

  genvar i,j;

  assign apb_slverr_o  = 1'b0;

  //assign rd_sgrant_ack = buf_wen && ~buf_cen;                                                 // Read  grant acknowledgement when the buffer is accessed in read mode
  assign wr_sgrant_ack = |wr_grant_ack;                                                       // Write grant acknowledgement output by the buffer address generator 
  assign rd_sgrant_ack = ~wr_sgrant_ack & udma_vtransfer_adc;

  assign buf_rwn       = ~wr_anyGrant;                                                        // Buffer in write mode if at least one ADC has been granted
  assign buf_wen       = ~wr_sgrant_ack;                                                      // Buffer write enable
  assign buf_cen       = ~(wr_sgrant_ack | udma_ce);                                          // Buffer chip enable
  assign udma_valid    = buf_rwn ? rd_anyGrant : 1'b0;                                        // The input data at the uDMA interface is valid if in read mode and one ADC is requesting the access

  /* Multiplexers -- choose address and data for the SRAM buffer and the L2 memory */
  assign buf_grant     =  buf_rwn ? adc_rd_grants : adc_wr_grants;                            // GRANT MUX -- grants ADC granted by READ  arbiter if in read  mode
                                                                                              //           -- grants ADC granted by WRITE arbiter if in write mode
  // check: the silencing below schould not be necessary
  assign buf_wdata     = ~buf_rwn ? adc_rx_data_sync[adc_log_wr_grant] : '0;                  // DATA  MUX -- if in write mode
  assign buf_addr_sel  = (wr_anyGrant || rd_anyGrant) ? buf_addr[buf_log_grant] : '0;         // BUF ADDR MUX


  always_comb begin: idExtractor
    ch_id = '0;
    CH_ID_LSB   = AFE_RX_CHID_LSB[r_buf_log_grant];
    CH_ID_WIDTH = AFE_RX_CHID_WIDTH[r_buf_log_grant];
    for(int l=0; l<ADC_DATA_WIDTH; l++) begin
      if(l > (CH_ID_LSB-1) && l<(CH_ID_LSB+CH_ID_WIDTH))
        ch_id[l-CH_ID_LSB] = buf_rdata[l];
    end
  end

  always_comb begin: subidExtractor
    subch_id = '0;
    if (AFE_RX_TYPE[r_buf_log_grant] == 1) begin
      SUBCH_ID_LSB   = AFE_RX_SUBCHID_LSB[r_buf_log_grant];
      SUBCH_ID_WIDTH = AFE_RX_SUBCHID_WIDTH[r_buf_log_grant];
      for(int l=0; l<ADC_DATA_WIDTH; l++) begin
        if(l > (SUBCH_ID_LSB-1) && l<(SUBCH_ID_LSB+SUBCH_ID_WIDTH))
          subch_id[l-SUBCH_ID_LSB] = buf_rdata[l];
      end
    end
  end

  always_comb begin: rd_req_proc
    if(buf_rwn) begin // in read mode
      adc_rd_req = udma_read_valid;
    end 
    else begin // in write mode
      adc_rd_req = '0;
    end
  end

  always_comb begin
    buf_rdata_mask = buf_rdata;

    case (data_mask_mode[r_buf_log_grant])
      2'b01: begin
        buf_rdata_mask = buf_rdata & AFE_RX_MASK_FL[r_buf_log_grant];
      end
      2'b10: begin
        buf_rdata_mask = buf_rdata & AFE_RX_MASK_PL[r_buf_log_grant];
      end
    endcase
  end

  generate 
    for(i=0; i<NUM_AFE_RX; i++) begin: adc_flag_assign
      if (AFE_FLAG_MASK[i] == 1) begin
        if (AFE_RX_FLAG_WIDTH[i] < AFE_RX_MAX_FLAG_WIDTH)
          assign adc_flag_data[i][AFE_RX_MAX_FLAG_WIDTH-1 : AFE_RX_FLAG_WIDTH[i]] = '0;
      
        assign adc_flag_data[i][0 +: AFE_RX_FLAG_WIDTH[i]] = (udma_valid && (r_buf_log_grant == i)) ? buf_rdata[AFE_RX_FLAG_LSB[i] +: AFE_RX_FLAG_WIDTH[i]] : '0;
        assign adc_flag_data_vld[i] = (udma_valid && udma_vtransfer_adc && (r_buf_log_grant == i));
      end
      else begin
        assign adc_flag_data[i]     = '0;
        assign adc_flag_data_vld[i] = 1'b0;
      end
    end
  endgenerate

  always_comb begin: rx_addr_proc
    rx_addr_sel = '0;

    if (udma_vtransfer_adc) begin
      if (AFE_RX_TYPE[r_buf_log_grant] == 1)
        rx_addr_sel  = rx_addr[r_buf_log_grant][ch_id] + subch_id*rx_stride_size[r_buf_log_grant][ch_id];
      else
        rx_addr_sel  = rx_addr[r_buf_log_grant][ch_id];
    end
  end

  /* Assign the valid transfer signal to the selected AFE and channel */
  always_comb begin: valid_trans_proc
    udma_vtransfer = '0;
    if(udma_vtransfer_adc)
      udma_vtransfer[r_buf_log_grant][ch_id] = 1'b1;
  end


  /* Configuration Address extractor -- to know which ADC is targeted */
  assign cfg_valid      = apb_sel_i & apb_en_i;
  assign cfg_data_in    = apb_wdata_i;
  assign cfg_addr_in    = apb_address_i[12:2];
  assign cfg_rwn_in     = ~apb_write_i;

  assign s_cfg_adc_sel  = cfg_valid ? cfg_addr_in[10:8] : '0;

  always_comb begin
    cfg_valid_in      = '0;
    cfg_valid_shared  = 1'b0;

    if(cfg_valid) begin
      if (s_cfg_adc_sel == 3'b111)
        cfg_valid_shared  = 1'b1;
      else
        cfg_valid_in[s_cfg_adc_sel] = 1'b1;
    end
  end

  always_comb begin : cfg_to_apb_rdata
    apb_rdata_o = 'h0;
    apb_ready_o = 1'b0;
    
    for (int i=0; i<NUM_AFE_RX; i++) begin
      if (s_cfg_adc_sel == i) begin
        apb_rdata_o = cfg_data_out[i];
        apb_ready_o = cfg_ready_out[i];
      end
    end

    if (s_cfg_adc_sel == 3'b111) begin
      apb_rdata_o = cfg_data_shared;
      apb_ready_o = cfg_ready_shared;
    end
  end

  always_comb begin: log_proc
    adc_log_rd_grant = '0;
    adc_log_wr_grant = '0;
    buf_log_grant    = '0;

    for(int k=0; k<NUM_AFE_RX; k++) begin
      if (adc_rd_grants[k]) adc_log_rd_grant = k;
      if (adc_wr_grants[k]) adc_log_wr_grant = k;
      if (buf_grant[k])     buf_log_grant    = k;
    end
  end


  afe_ro_conf afe_ro_conf_i (
    .clk_i              ( clk_i            ),
    .rstn_i             ( rstn_i           ),
 
    .cfg_data_i         ( cfg_data_in      ),
    .cfg_addr_i         ( cfg_addr_in      ),
    .cfg_valid_i        ( cfg_valid_shared ),
    .cfg_rwn_i          ( cfg_rwn_in       ),
    .cfg_data_o         ( cfg_data_shared  ),
    .cfg_ready_o        ( cfg_ready_shared ),

    // RX registers configuration signals
    .cfg_udma_shtdwn_o  ( udma_shtdwn      )
  );

  generate 
    for(i=0; i<NUM_AFE_RX; i++) begin: adc_gen_loop

      case (AFE_RX_TYPE[i])
        0: begin : ADC_TOP_INST
          adc_top_type0 #(
            .L2_AWIDTH_NOAL  ( L2_AWIDTH_NOAL        ),
            .UDMA_TRANS_SIZE ( UDMA_TRANS_SIZE       ),
            .TRANS_SIZE      ( AFE_RX_TRANS_SIZE[i]  ),
            .BUF_AWIDTH      ( BUF_AWIDTH            ),
            .BUF_TRANS_SIZE  ( BUF_TRANS_SIZE        ),
            .ADC_DATA_WIDTH  ( ADC_DATA_WIDTH        ),
            .ADC_NUM_CHS     ( AFE_RX_NUM_CHS   [i]  ),
            .CH_ID_LSB       ( AFE_RX_CHID_LSB  [i]  ),
            .CH_ID_WIDTH     ( AFE_RX_CHID_WIDTH[i]  ),
            .MAX_CH_PER_ADC  ( AFE_RX_MAX_NUM_CH     ),
            .FEATURE_FLAG    ( AFE_FLAG_MASK[i]      ),
            .FLAG_WIDTH      ( AFE_RX_MAX_FLAG_WIDTH ),
            .FLAG_CHID_WIDTH ( LOG_MAX_CH_PER_ADC    )
          ) adc_top_i (
            .clk_i                ( clk_i                   ),
            .rstn_i               ( rstn_i                  ),
  
            .cfg_data_i           ( cfg_data_in             ),
            .cfg_addr_i           ( cfg_addr_in             ),
            .cfg_valid_i          ( cfg_valid_in[i]         ),
            .cfg_rwn_i            ( cfg_rwn_in              ),
            .cfg_data_o           ( cfg_data_out[i]         ),
            .cfg_ready_o          ( cfg_ready_out[i]        ),

            .data_mask_mode_o     ( data_mask_mode[i]       ),
  
            .buf_rwn_i            ( buf_rwn                 ),
            .adc_grant_i          ( buf_grant[i]            ),
            .ch_event_o           ( l2_half_event_o[AFE_RX_OFFS_L2CH[i] +: AFE_RX_NUM_L2CHS[i]] ),
            .buf_event_o          ( buf_event_o[i]          ),
            .flag_event_o         ( flag_event_o[i]         ),
            .rx_addr_o            ( rx_addr[i]              ),
            .buf_addr_o           ( buf_addr[i]             ),

            .flags_i              ( adc_flag_data[i]        ),
            .flags_valid_i        ( adc_flag_data_vld[i]    ),
            .flags_chid_i         ( ch_id                   ),

            .udma_vtransfer_i     ( udma_vtransfer[i]       ),
            .udma_read_valid_o    ( udma_read_valid[i]      ),
  
            .wr_grant_ack_o       ( wr_grant_ack[i]         ),
  
            .adc_rx_valid_async_i ( afe_data_valid_i[i]     ),
            .adc_rx_data_async_i  ( afe_data_i[i]           ),
            .adc_rx_valid_sync_o  ( adc_rx_valid_sync[i]    ),
            .adc_rx_data_sync_o   ( adc_rx_data_sync[i]     )
          );

          assign rx_stride_size[i] = '0;
        end
        
        1: begin : ADC_TOP_INST
          adc_top_type1 #(
            .L2_AWIDTH_NOAL     ( L2_AWIDTH_NOAL          ),
            .UDMA_TRANS_SIZE    ( UDMA_TRANS_SIZE         ),
            .TRANS_SIZE         ( AFE_RX_TRANS_SIZE[i]    ),
            .BUF_AWIDTH         ( BUF_AWIDTH              ),
            .BUF_TRANS_SIZE     ( BUF_TRANS_SIZE          ),
            .ADC_DATA_WIDTH     ( ADC_DATA_WIDTH          ),
            .ADC_NUM_CHS        ( AFE_RX_NUM_CHS   [i]    ),
            .STRIDE_SIZE        ( AFE_RX_STRIDE_SIZE      ),
            .ADC_SUBCH_ID_WIDTH ( AFE_RX_SUBCHID_WIDTH[i] ),
            .CH_ID_LSB          ( AFE_RX_CHID_LSB  [i]    ),
            .CH_ID_WIDTH        ( AFE_RX_CHID_WIDTH[i]    ),
            .MAX_CH_PER_ADC     ( AFE_RX_MAX_NUM_CH       ),
            .FEATURE_FLAG       ( AFE_FLAG_MASK[i]        ),
            .FLAG_WIDTH         ( AFE_RX_MAX_FLAG_WIDTH   ),
            .FLAG_CHID_WIDTH    ( LOG_MAX_CH_PER_ADC      ),
            .FLAG_FULL_ID_WIDTH ( LOG_MAX_CH_PER_ADC+LOG_MAX_SUBCH_PER_ADC )
          ) adc_top_i (
            .clk_i                ( clk_i                   ),
            .rstn_i               ( rstn_i                  ),
  
            .cfg_data_i           ( cfg_data_in             ),
            .cfg_addr_i           ( cfg_addr_in             ),
            .cfg_valid_i          ( cfg_valid_in[i]         ),
            .cfg_rwn_i            ( cfg_rwn_in              ),
            .cfg_data_o           ( cfg_data_out[i]         ),
            .cfg_ready_o          ( cfg_ready_out[i]        ),

            .data_mask_mode_o     ( data_mask_mode[i]       ),
  
            .buf_rwn_i            ( buf_rwn                 ),
            .adc_grant_i          ( buf_grant[i]            ),
            .ch_event_o           ( l2_half_event_o[AFE_RX_OFFS_L2CH[i] +: AFE_RX_NUM_L2CHS[i]] ),
            .buf_event_o          ( buf_event_o[i]          ),
            .flag_event_o         ( flag_event_o[i]         ),
            .rx_stride_size_o     ( rx_stride_size[i]       ),
            .rx_addr_o            ( rx_addr[i]              ),
            .buf_addr_o           ( buf_addr[i]             ),

            .flags_i              ( adc_flag_data[i]        ),
            .flags_valid_i        ( adc_flag_data_vld[i]    ),
            .flags_chid_i         ( ch_id                   ),
            .flags_full_id_i      ( {ch_id, subch_id}       ),

            .udma_subch_id_i      ( subch_id                ),
            .udma_vtransfer_i     ( udma_vtransfer[i]       ),
            .udma_read_valid_o    ( udma_read_valid[i]      ),
  
            .wr_grant_ack_o       ( wr_grant_ack[i]         ),
  
            .adc_rx_valid_async_i ( afe_data_valid_i[i]     ),
            .adc_rx_data_async_i  ( afe_data_i[i]           ),
            .adc_rx_valid_sync_o  ( adc_rx_valid_sync[i]    ),
            .adc_rx_data_sync_o   ( adc_rx_data_sync[i]     )
          );
        end

        2: begin : ADC_TOP_INST
          adc_top_type2 #(
            .L2_AWIDTH_NOAL  ( L2_AWIDTH_NOAL        ),
            .UDMA_TRANS_SIZE ( UDMA_TRANS_SIZE       ),
            .TRANS_SIZE      ( AFE_RX_TRANS_SIZE[i]  ),
            .BUF_AWIDTH      ( BUF_AWIDTH            ),
            .BUF_TRANS_SIZE  ( BUF_TRANS_SIZE        ),
            .ADC_DATA_WIDTH  ( ADC_DATA_WIDTH        ),
            .ADC_NUM_CHS     ( AFE_RX_NUM_CHS   [i]  ),
            .L2_NUM_CHS      ( AFE_RX_NUM_L2CHS [i]  ),
            .CH_ID_LSB       ( AFE_RX_CHID_LSB  [i]  ),
            .CH_ID_WIDTH     ( AFE_RX_CHID_WIDTH[i]  ),
            .MAX_CH_PER_ADC  ( AFE_RX_MAX_NUM_CH     ),
            .FEATURE_FLAG    ( AFE_FLAG_MASK[i]      ),
            .FLAG_WIDTH      ( AFE_RX_MAX_FLAG_WIDTH ),
            .FLAG_CHID_WIDTH ( LOG_MAX_CH_PER_ADC    )
          ) adc_top_i (
            .clk_i                ( clk_i                   ),
            .rstn_i               ( rstn_i                  ),

            .cfg_data_i           ( cfg_data_in             ),
            .cfg_addr_i           ( cfg_addr_in             ),
            .cfg_valid_i          ( cfg_valid_in[i]         ),
            .cfg_rwn_i            ( cfg_rwn_in              ),
            .cfg_data_o           ( cfg_data_out[i]         ),
            .cfg_ready_o          ( cfg_ready_out[i]        ),

            .data_mask_mode_o     ( data_mask_mode[i]       ),

            .buf_rwn_i            ( buf_rwn                 ),
            .adc_grant_i          ( buf_grant[i]            ),
            .ch_event_o           ( l2_half_event_o[AFE_RX_OFFS_L2CH[i] +: AFE_RX_NUM_L2CHS[i]] ),
            .buf_event_o          ( buf_event_o[i]          ),
            .flag_event_o         ( flag_event_o[i]         ),
            .rx_addr_o            ( rx_addr[i]              ),
            .buf_addr_o           ( buf_addr[i]             ),

            .flags_i              ( adc_flag_data[i]        ),
            .flags_valid_i        ( adc_flag_data_vld[i]    ),
            .flags_chid_i         ( ch_id                   ),

            .udma_vtransfer_i     ( udma_vtransfer[i]       ),
            .udma_read_valid_o    ( udma_read_valid[i]      ),

            .wr_grant_ack_o       ( wr_grant_ack[i]         ),

            .adc_rx_valid_async_i ( afe_data_valid_i[i]     ),
            .adc_rx_data_async_i  ( afe_data_i[i]           ),
            .adc_rx_valid_sync_o  ( adc_rx_valid_sync[i]    ),
            .adc_rx_data_sync_o   ( adc_rx_data_sync[i]     )
          );

          assign rx_stride_size[i] = '0;
        end
      endcase
    end
  endgenerate

  /*
  logic DEBUG_mult_wr_req, DEBUG_mult_rd_req;
  logic DEBUG_rd_wr_simul;
  logic DEBUG_read_after_write;

  assign DEBUG_mult_wr_req = (r_adc_wr_req != 4'b0000) && (r_adc_wr_req != 4'b0001) && (r_adc_wr_req != 4'b0010) && (r_adc_wr_req != 4'b0100) && (r_adc_wr_req != 4'b1000);
  assign DEBUG_mult_rd_req = (adc_rd_req   != 4'b0000) && (adc_rd_req   != 4'b0001) && (adc_rd_req   != 4'b0010) && (adc_rd_req   != 4'b0100) && (adc_rd_req   != 4'b1000);
  assign DEBUG_read_after_write = (buf_rwn && ~r_buf_rwn && ~r_udma_read_valid);
  assign DEBUG_rd_wr_simul = (r_adc_wr_req != 4'b0000) && (adc_rd_req   != 4'b0000);
  */

  // arbiter when buffer in write mode
  udma_arbiter #(
    .N ( NUM_AFE_RX ),
    .S ( 3          )
  ) write_arbiter_i (
    .clk_i       ( clk_i  ),
    .rstn_i      ( rstn_i ),

    .req_i       ( r_adc_wr_req  ),
    .grant_o     ( adc_wr_grants ),
    .grant_ack_i ( wr_sgrant_ack ),
    .anyGrant_o  ( wr_anyGrant   )
  );

  // arbiter when buffer in read mode
  udma_arbiter #(
    .N ( NUM_AFE_RX ),
    .S ( 3          )
  ) read_arbiter_i (
    .clk_i       ( clk_i  ),
    .rstn_i      ( rstn_i ),

    .req_i       ( adc_rd_req      ),  
    .grant_o     ( adc_rd_grants   ),
    .grant_ack_i ( rd_sgrant_ack   ),
    .anyGrant_o  ( rd_anyGrant     )
  );


  // Protocol IF to communicate with the remaining part of the uDMA
  afe_udma_if #(
    .L2_DATA_WIDTH  ( 32             ),
    .L2_AWIDTH_NOAL ( L2_AWIDTH_NOAL )
  ) udma_if_i (
    .clk_i          ( clk_i              ),
    .rstn_i         ( rstn_i             ),

    .ro_valid_i     ( udma_valid         ),
    .ro_vtransfer_o ( udma_vtransfer_adc ),
    .ro_buf_ce_o    ( udma_ce            ),
    .ro_rdata_i     ( buf_rdata_mask     ),
    .ro_raddr_i     ( rx_addr_sel        ),

    .udma_shtdwn_i  ( udma_shtdwn        ),
    .udma_ready_i   ( udma_ready_i       ),
    .udma_valid_o   ( udma_valid_o       ),
    .udma_wdata_o   ( udma_data_o        ),
    .udma_waddr_o   ( udma_addr_o        )
  );


  sram_buffer #(
    .ADC_DATA_WIDTH ( ADC_DATA_WIDTH ),
    .ADDR_WIDTH     ( BUF_AWIDTH     )
  ) buffer_i (
    .clk_i  ( clk_i         ),

    .cen_i  ( buf_cen       ),
    .wen_i  ( buf_wen       ),
    .data_i ( buf_wdata     ),
    .addr_i ( buf_addr_sel  ),
    .data_o ( buf_rdata     )
  );

  /* ADC requests assignment -----------------------------------------*/
  always_comb begin : adc_req_buff
    adc_wr_req  = r_adc_wr_req | adc_rx_valid_sync;
    for (int i=0; i<NUM_AFE_RX; i++) begin
      if (wr_sgrant_ack && (adc_log_wr_grant == i))
        adc_wr_req[i] = 1'b0;
    end
  end

  always_ff @(posedge clk_i, negedge rstn_i) begin
    if(~rstn_i) begin
      r_buf_log_grant    <= '0;
      r_adc_wr_req       <= '0;
      r_buf_rwn          <= '0;
      r_udma_read_valid  <= '0;
    end
    else begin
      r_buf_log_grant    <= buf_log_grant;
      r_adc_wr_req       <= adc_wr_req;
      r_buf_rwn          <= buf_rwn; 
      r_udma_read_valid  <= udma_read_valid;
    end
  end

endmodule