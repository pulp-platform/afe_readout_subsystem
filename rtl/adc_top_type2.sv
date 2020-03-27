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

module adc_top_type2 #(
  parameter L2_AWIDTH_NOAL  = 12,
  parameter UDMA_TRANS_SIZE = 16,
  parameter TRANS_SIZE      = 16,
  parameter BUF_AWIDTH      = 10,
  parameter BUF_TRANS_SIZE  = 16,
  parameter ADC_DATA_WIDTH  = 32,
  parameter ADC_NUM_CHS     = 8,
  parameter L2_NUM_CHS      = ADC_NUM_CHS,
  parameter CH_ID_LSB       = 28,
  parameter CH_ID_WIDTH     = 4,
  parameter MAX_CH_PER_ADC  = 32,
  parameter FEATURE_FLAG    = 0,
  parameter FLAG_WIDTH      = 1,
  parameter FLAG_CHID_WIDTH = 4
) (
  input  logic                      clk_i,      
  input  logic                      rstn_i,           

  input  logic               [31:0] cfg_data_i,
  input  logic               [10:0] cfg_addr_i,
  input  logic                      cfg_valid_i,
  input  logic                      cfg_rwn_i,
  output logic               [31:0] cfg_data_o,
  output logic                      cfg_ready_o,

  output logic                [1:0] data_mask_mode_o,

  input logic                       buf_rwn_i,
  input  logic                      adc_grant_i,
  output logic     [L2_NUM_CHS-1:0] ch_event_o,
  output logic                      buf_event_o,            // the filling level of the buffer has been reached
  output logic                      flag_event_o,

  output logic                         [BUF_AWIDTH-1:0] buf_addr_o,
  output logic [MAX_CH_PER_ADC-1:0][L2_AWIDTH_NOAL-1:0] rx_addr_o,

  input  logic                        flags_valid_i,
  input  logic       [FLAG_WIDTH-1:0] flags_i,
  input  logic  [FLAG_CHID_WIDTH-1:0] flags_chid_i,

  // UDMA Protocol IF signals
  input  logic [MAX_CH_PER_ADC-1:0] udma_vtransfer_i,
  output logic                      udma_read_valid_o,

  // Grant acknowledgements for the write arbiter
  output logic                      wr_grant_ack_o,

  // ADC signals
  input  logic                      adc_rx_valid_async_i,
  input  logic [ADC_DATA_WIDTH-1:0] adc_rx_data_async_i,
  output logic                      adc_rx_valid_sync_o,
  output logic [ADC_DATA_WIDTH-1:0] adc_rx_data_sync_o 
);

  // RX registers configuration signals
  logic  [L2_NUM_CHS-1:0] [L2_AWIDTH_NOAL-1:0] int_rx_startaddr;
  logic  [L2_NUM_CHS-1:0][UDMA_TRANS_SIZE-1:0] int_rx_size;
  logic  [L2_NUM_CHS-1:0]                      int_rx_continuous;
  logic  [L2_NUM_CHS-1:0]    [CH_ID_WIDTH-1:0] int_rx_adc_chid;
  logic  [L2_NUM_CHS-1:0]                      int_rx_cfg_en;
  logic  [L2_NUM_CHS-1:0]                      int_rx_clr;
  logic  [L2_NUM_CHS-1:0]                      int_rx_ch_en;
  logic  [L2_NUM_CHS-1:0] [L2_AWIDTH_NOAL-1:0] int_rx_curr_addr;
  logic  [L2_NUM_CHS-1:0][UDMA_TRANS_SIZE-1:0] int_rx_wr_ptr;
  logic  [L2_NUM_CHS-1:0][UDMA_TRANS_SIZE-1:0] int_rx_bytes_left;

  logic                  [UDMA_TRANS_SIZE-1:0] int_rx_wr_ptr_flag;

  // Buffer registers configuration signals
  logic                       [BUF_AWIDTH-1:0] int_buf_startaddr;
  logic                   [BUF_TRANS_SIZE-1:0] int_buf_size;
  logic                   [BUF_TRANS_SIZE-1:0] int_buf_fillinglevel;
  logic                                        int_buf_continuous;
  logic                                        int_buf_cfg_clr;
  logic                                        int_buf_cfg_en;
  logic                                        int_buf_en;
  logic                       [BUF_AWIDTH-1:0] int_buf_curr_addr;
  logic                   [BUF_TRANS_SIZE-1:0] int_buf_bytes_left;

  logic                      [ADC_NUM_CHS-1:0] int_buf_ch_mask;
  logic                                        int_buf_en_mode;
  logic                      [CH_ID_WIDTH-1:0] int_buf_en_chid;


  // UDMA Protocol IF internal signals
  logic                           buf_udma_vtransfer;
  logic         [ADC_NUM_CHS-1:0] udma_vtransfer;
  logic [$clog2(ADC_NUM_CHS)-1:0] udma_vtransfer_log;
  logic          [L2_NUM_CHS-1:0] udma_vtransfer_L2;

  logic                           flag_event;
  logic                           flag_event_en;
  logic                    [31:0] flag_data;

  /* Outputs assignments */
  always_comb begin
    rx_addr_o         = '0;
    
    for(int j=0; j<MAX_CH_PER_ADC; j++) begin
      if (j < ADC_NUM_CHS) begin
        for(int k=0; k<L2_NUM_CHS; k++) begin
          if (int_rx_adc_chid[k] == j) begin
            rx_addr_o[j]         = int_rx_curr_addr[k];
            break;
          end
        end
      end 
    end
  end

  always_comb begin
    int_rx_wr_ptr_flag = '0;

    for(int k=0; k<L2_NUM_CHS; k++) begin
      if (int_rx_adc_chid[k] == flags_chid_i) begin
        int_rx_wr_ptr_flag  = int_rx_wr_ptr[k];
        break;
      end
    end
  end

  assign buf_addr_o     = int_buf_curr_addr;
  assign flag_event_o   = flag_event;

  assign udma_vtransfer     = udma_vtransfer_i[ADC_NUM_CHS-1:0];
  assign buf_udma_vtransfer = |udma_vtransfer;                          // the ADC received the valid transfer signal if at least one channel has received it

  generate
    if (FEATURE_FLAG == 1) begin
      assign flag_event = flags_valid_i & flag_event_en & |flags_i;

      assign flag_data[31  : 24+FLAG_CHID_WIDTH]  = '0;
      assign flag_data[24 +: FLAG_CHID_WIDTH]     = (flag_event) ? flags_chid_i : '0;
      assign flag_data[23  : 16+FLAG_WIDTH]       = '0;
      assign flag_data[16 +: FLAG_WIDTH]          = (flag_event) ? flags_i      : '0;
      assign flag_data[15  : (TRANS_SIZE-2)]      = '0;
      assign flag_data[0  +: (TRANS_SIZE-2)]      = (flag_event) ? int_rx_wr_ptr_flag[TRANS_SIZE-1:2] : '0;
    end
    else begin
      assign flag_event = 1'b0;
      assign flag_data  = '0;
    end
  endgenerate

  // must be changed:
  // rx_addr for each ADC ch generated
  // udma_vtransfer routed to correct L2 addr gen

  onehot_to_bin #( .ONEHOT_WIDTH (ADC_NUM_CHS) )
  onehot_to_bin_vtransf_i (
    .onehot ( udma_vtransfer     ),
    .bin    ( udma_vtransfer_log )
  );

  always_comb begin
    udma_vtransfer_L2 = '0;
    if (|udma_vtransfer) begin
      for(int k=0; k<L2_NUM_CHS; k++) begin
        if (int_rx_adc_chid[k] == udma_vtransfer_log)
          udma_vtransfer_L2[k] = 1'b1;
      end
    end
  end

  adc_sync_if #(
    .ADC_DATA_WIDTH ( ADC_DATA_WIDTH  ),
    .ADC_NUM_CHS    ( ADC_NUM_CHS     ),
    .ADC_CHID_LSB   ( CH_ID_LSB       ),
    .ADC_CHID_WIDTH ( CH_ID_WIDTH     )
  ) adc_sync 
  (
    .clk_i                ( clk_i       ),
    .rstn_i               ( rstn_i      ),

    .adc_ch_mask_i        ( int_buf_ch_mask      ),
    .buf_cfg_en_i         ( int_buf_cfg_en       ),
    .buf_cfg_clr_i        ( int_buf_cfg_clr      ),
    .buf_cfg_mode_i       ( int_buf_en_mode      ),
    .buf_cfg_en_chid_i    ( int_buf_en_chid      ),

    .adc_rx_valid_async_i ( adc_rx_valid_async_i ),
    .adc_rx_valid_sync_o  ( adc_rx_valid_sync_o  ),
    .adc_rx_data_async_i  ( adc_rx_data_async_i  ),
    .adc_rx_data_sync_o   ( adc_rx_data_sync_o   )
  );


  adc_reg_if_type2 #(
    .L2_AWIDTH_NOAL  ( L2_AWIDTH_NOAL  ),
    .UDMA_TRANS_SIZE ( UDMA_TRANS_SIZE ),
    .TRANS_SIZE      ( TRANS_SIZE      ),
    .ADC_NUM_CHS     ( ADC_NUM_CHS     ),
    .ADC_CHID_WIDTH  ( CH_ID_WIDTH     ),
    .L2_NUM_CHS      ( L2_NUM_CHS      ),
    .BUF_AWIDTH      ( BUF_AWIDTH      ),
    .BUF_TRANS_SIZE  ( BUF_TRANS_SIZE  )
  ) adc_reg
  (
    .clk_i                ( clk_i                  ),
    .rstn_i               ( rstn_i                 ),
 
    .cfg_data_i           ( cfg_data_i             ),
    .cfg_addr_i           ( cfg_addr_i             ),
    .cfg_valid_i          ( cfg_valid_i            ),
    .cfg_rwn_i            ( cfg_rwn_i              ),
    .cfg_data_o           ( cfg_data_o             ),
    .cfg_ready_o          ( cfg_ready_o            ),

    .cfg_flag_event_en_o  ( flag_event_en          ),
    .cfg_flag_event_i     ( flag_event             ),
    .cfg_flag_data_i      ( flag_data              ),

    // RX registers configuration signals
    .cfg_rx_startaddr_o   ( int_rx_startaddr       ),
    .cfg_rx_size_o        ( int_rx_size            ),
    .cfg_rx_continuous_o  ( int_rx_continuous      ),
    .cfg_rx_adc_chid_o    ( int_rx_adc_chid        ),
    .cfg_rx_en_o          ( int_rx_cfg_en          ),
    .cfg_rx_clr_o         ( int_rx_clr             ),
    .cfg_rx_en_i          ( int_rx_ch_en           ),
    .cfg_rx_curr_addr_i   ( int_rx_curr_addr       ),
    .cfg_rx_bytes_left_i  ( int_rx_bytes_left      ),

    // BUF registers configuration signals
    .cfg_buf_startaddr_o  ( int_buf_startaddr      ),
    .cfg_buf_size_o       ( int_buf_size           ),
    .cfg_buf_flevel_o     ( int_buf_fillinglevel   ),
    .cfg_buf_continuous_o ( int_buf_continuous     ),
    .cfg_buf_en_o         ( int_buf_cfg_en         ),
    .cfg_buf_clr_o        ( int_buf_cfg_clr        ),
    .cfg_buf_en_i         ( int_buf_en             ),
    .cfg_buf_curr_addr_i  ( int_buf_curr_addr      ),
    .cfg_buf_bytes_left_i ( int_buf_bytes_left     ),

    .cfg_buf_ch_mask_o    ( int_buf_ch_mask        ),
    .cfg_buf_en_mode_o    ( int_buf_en_mode        ),
    .cfg_buf_en_chid_o    ( int_buf_en_chid        ),
    .cfg_data_mask_mode_o ( data_mask_mode_o       )
  );


  genvar i;
  generate 
    for(i=0;i<L2_NUM_CHS;i++) begin
      afe_l2_addr_gen #(
        .AWIDTH       ( L2_AWIDTH_NOAL  ),
        .TRANS_SIZE   ( UDMA_TRANS_SIZE )
      ) l2_addr_gen (
        .clk_i             ( clk_i  ),
        .rstn_i            ( rstn_i ),
        .cfg_startaddr_i   ( int_rx_startaddr[i]    ),
        .cfg_size_i        ( int_rx_size[i]         ),
        .cfg_continuous_i  ( int_rx_continuous[i]   ),
        .cfg_en_i          ( int_rx_cfg_en[i]       ),
        .cfg_clr_i         ( int_rx_clr[i]          ),
        .cfg_curr_addr_o   ( int_rx_curr_addr[i]    ),
        .cfg_wr_ptr_o      ( int_rx_wr_ptr[i]       ),
        .cfg_bytes_left_o  ( int_rx_bytes_left[i]   ),
        .cfg_en_o          ( int_rx_ch_en[i]        ),
        .ch_event_o        ( ch_event_o[i]          ),
        .udma_vtransfer_i  ( udma_vtransfer_L2[i]   )
      );
    end
  endgenerate


  afe_buf_addr_gen #(
    .AWIDTH       ( BUF_AWIDTH     ),
    .TRANS_SIZE   ( BUF_TRANS_SIZE )
  ) buf_addr_gen (
    .clk_i             ( clk_i  ),
    .rstn_i            ( rstn_i ),

    .cfg_startaddr_i   ( int_buf_startaddr   ),
    .cfg_size_i        ( int_buf_size        ),
    .cfg_continuous_i  ( int_buf_continuous  ),
    .cfg_flevel_i      ( int_buf_fillinglevel),
    .cfg_en_i          ( int_buf_cfg_en      ),
    .cfg_clr_i         ( int_buf_cfg_clr     ),
    .cfg_curr_addr_o   ( int_buf_curr_addr   ),
    .cfg_bytes_left_o  ( int_buf_bytes_left  ),
    .cfg_en_o          ( int_buf_en          ),

    .buf_rwn_i         ( buf_rwn_i           ),
    .adc_grant_i       ( adc_grant_i         ),
    .buf_event_o       ( buf_event_o         ),

    .write_grant_ack_o ( wr_grant_ack_o      ),

    .udma_vtransfer_i  ( buf_udma_vtransfer  ),
    .udma_read_valid_o ( udma_read_valid_o   )
  );


endmodule