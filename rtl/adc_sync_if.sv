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

module adc_sync_if #(
  parameter ADC_DATA_WIDTH  = 32,
  parameter ADC_NUM_CHS     = 8,
  parameter ADC_CHID_LSB    = 28,
  parameter ADC_CHID_WIDTH  = 4
) (
  input  logic                      clk_i,      // master clock
  input  logic                      rstn_i,     // asynchronous active low reset

  input  logic    [ADC_NUM_CHS-1:0] adc_ch_mask_i,

  input  logic                      buf_cfg_en_i,
  input  logic                      buf_cfg_clr_i,
  input  logic                      buf_cfg_mode_i,
  input  logic [ADC_CHID_WIDTH-1:0] buf_cfg_en_chid_i,

  // ADC signals
  input  logic                      adc_rx_valid_async_i,
  output logic                      adc_rx_valid_sync_o,
  input  logic [ADC_DATA_WIDTH-1:0] adc_rx_data_async_i,
  output logic [ADC_DATA_WIDTH-1:0] adc_rx_data_sync_o 
);

  logic  [ADC_CHID_WIDTH-1:0] adc_ch_id;

  logic                 [2:0] adc_data_valid_sync;
  logic  [ADC_DATA_WIDTH-1:0] adc_data_sync;
  
  logic     adc_vld_edge, adc_vld_edge_del;
  logic     adc_ro_en;

  enum logic [1:0] { WAIT, ARMED, ENABLED } adc_ro_en_cs, adc_ro_en_ns;

  assign adc_ch_id  = adc_data_sync[ADC_CHID_LSB +: ADC_CHID_WIDTH];

  assign adc_rx_valid_sync_o = adc_vld_edge_del & adc_ch_mask_i[adc_ch_id] & adc_ro_en;
  assign adc_rx_data_sync_o  = adc_data_sync;

  assign adc_vld_edge = adc_data_valid_sync[1] & ~adc_data_valid_sync[2];


  always_comb begin
    adc_ro_en_ns  = adc_ro_en_cs;
    adc_ro_en     = 1'b1;

    case (adc_ro_en_cs)
      WAIT: begin
        adc_ro_en     = 1'b0;

        if (buf_cfg_en_i) begin
          if (buf_cfg_mode_i)
            adc_ro_en_ns = ARMED;
          else
            adc_ro_en_ns = ENABLED;
        end
      end
      ARMED: begin
        adc_ro_en     = 1'b0;

        if (adc_vld_edge_del && (buf_cfg_en_chid_i == adc_ch_id)) begin
          adc_ro_en     = 1'b1;
          adc_ro_en_ns  = ENABLED;
        end

        if (buf_cfg_clr_i)
          adc_ro_en_ns = WAIT;
      end
      ENABLED: begin
        if (buf_cfg_clr_i)
          adc_ro_en_ns = WAIT;
      end
    endcase
  end

  // sync & edge detect of adc_data_valid_i
  always_ff @(posedge clk_i, negedge rstn_i) begin
    if (~rstn_i) begin
      adc_vld_edge_del        <= 1'b0;
      adc_data_valid_sync     <= '0;
      adc_data_sync           <= '0;
      adc_ro_en_cs            <= WAIT;
    end
    else begin
      adc_data_valid_sync[0]  <= adc_rx_valid_async_i;
      adc_data_valid_sync[1]  <= adc_data_valid_sync[0];
      adc_data_valid_sync[2]  <= adc_data_valid_sync[1];
      adc_vld_edge_del        <= adc_vld_edge;
      adc_ro_en_cs            <= adc_ro_en_ns;

      if (adc_vld_edge) 
        adc_data_sync         <= adc_rx_data_async_i;

    end
  end

endmodule
