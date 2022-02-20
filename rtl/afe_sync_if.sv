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

// Authors: Florian Glaser <glaserf@iis.ee.ethz.ch>

module afe_sync_if #(
  parameter AFE_DATA_WIDTH  = 32,
  parameter AFE_NUM_CHS     = 8,
  parameter AFE_CHID_LSB    = 28,
  parameter AFE_CHID_WIDTH  = 4
) (
  input  logic                      clk_i,
  input  logic                      rst_ni,
  input  logic                      test_mode_i,

  input  logic    [AFE_NUM_CHS-1:0] afe_ch_mask_i,
  input  logic                      afe_ch_en_mode_i,
  input  logic [AFE_CHID_WIDTH-1:0] afe_ch_en_chid_i,

  input  logic                      buff_cfg_en_i,
  input  logic                      buff_cfg_clr_i,

  // AFE valid and data, to be synched
  input  logic                      afe_valid_async_i,
  output logic                      afe_valid_sync_o,
  input  logic [AFE_DATA_WIDTH-1:0] afe_data_async_i,
  output logic [AFE_DATA_WIDTH-1:0] afe_data_sync_o,

  input  logic                      buff_ready_i
);

  logic  [AFE_CHID_WIDTH-1:0] afe_ch_id;

  logic                 [2:0] afe_data_valid_sync;
  logic  [AFE_DATA_WIDTH-1:0] afe_data_sync;
  
  logic     afe_valid_q, afe_valid_n;
  logic     afe_vld_edge, afe_vld_edge_del;
  logic     afe_ro_en;

  enum logic [1:0] { WAIT, ARMED, ENABLED } afe_ro_en_cs, afe_ro_en_ns;

  assign afe_ch_id = afe_data_sync[AFE_CHID_LSB +: AFE_CHID_WIDTH];

  assign afe_valid_sync_o = afe_valid_q;
  assign afe_data_sync_o  = afe_data_sync;

  // edge detection on both edges
  assign afe_vld_edge = (afe_data_valid_sync[1] & ~afe_data_valid_sync[2]) | (~afe_data_valid_sync[1] & afe_data_valid_sync[2]);


  always_comb begin
    afe_ro_en_ns  = afe_ro_en_cs;
    afe_ro_en     = 1'b1;

    case (afe_ro_en_cs)

      WAIT: begin
        afe_ro_en     = 1'b0;

        if (buff_cfg_en_i) begin
          if (afe_ch_en_mode_i)
            afe_ro_en_ns = ARMED;
          else
            afe_ro_en_ns = ENABLED;
        end
      end

      ARMED: begin
        afe_ro_en     = 1'b0;

        if (afe_vld_edge_del && (afe_ch_en_chid_i == afe_ch_id)) begin
          afe_ro_en     = 1'b1;
          afe_ro_en_ns  = ENABLED;
        end

        if (buff_cfg_clr_i)
          afe_ro_en_ns = WAIT;
      end

      ENABLED: begin
        if (buff_cfg_clr_i)
          afe_ro_en_ns = WAIT;
      end

    endcase
  end

  always_comb begin
    afe_valid_n = afe_valid_q;

    if (afe_vld_edge_del & afe_ch_mask_i[afe_ch_id] & afe_ro_en)
      afe_valid_n = 1'b1;
    else if (afe_valid_q & buff_ready_i)
      afe_valid_n = 1'b0;
  end

  // sync & edge detect of afe_data_valid_i
  always_ff @(posedge clk_i, negedge rst_ni) begin
    if (~rst_ni) begin
      afe_valid_q             <= 1'b0;
      afe_vld_edge_del        <= 1'b0;
      afe_data_valid_sync     <= '0;
      afe_data_sync           <= '0;
      afe_ro_en_cs            <= WAIT;
    end
    else begin
      afe_data_valid_sync[0]  <= afe_valid_async_i;
      afe_data_valid_sync[1]  <= afe_data_valid_sync[0];
      afe_data_valid_sync[2]  <= afe_data_valid_sync[1];
      afe_valid_q             <= afe_valid_n;
      afe_vld_edge_del        <= afe_vld_edge;
      afe_ro_en_cs            <= afe_ro_en_ns;

      if (afe_vld_edge) 
        afe_data_sync <= afe_data_async_i;

    end
  end

endmodule
