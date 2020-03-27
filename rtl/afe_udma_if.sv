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

module afe_udma_if 
#(
  parameter L2_DATA_WIDTH = 32,
  parameter L2_AWIDTH_NOAL = 12
) (
  input  logic             clk_i,
  input  logic             rstn_i,

  input  logic                      ro_valid_i,
  output logic                      ro_vtransfer_o,
  output logic                      ro_buf_ce_o,
  input  logic  [L2_DATA_WIDTH-1:0] ro_rdata_i,
  input  logic [L2_AWIDTH_NOAL-1:0] ro_raddr_i,

  input  logic                      udma_shtdwn_i,

  input  logic                      udma_ready_i,
  output logic                      udma_valid_o,
  output logic  [L2_DATA_WIDTH-1:0] udma_wdata_o,
  output logic [L2_AWIDTH_NOAL-1:0] udma_waddr_o
);

  enum logic [1:0] {IDLE, SAMPLE, RUN} state, next_state;

  logic  [L2_DATA_WIDTH-1:0] r_data, s_data;
  logic [L2_AWIDTH_NOAL-1:0] r_addr, s_addr;


  assign udma_wdata_o  = r_data;
  assign udma_waddr_o  = r_addr;

  always_comb begin
    ro_buf_ce_o     = 1'b0;
    ro_vtransfer_o  = 1'b0;

    udma_valid_o    = 1'b0;
    s_data          = r_data;
    s_addr          = r_addr;

    next_state      = state;

    case(state)
      IDLE: begin
        if (ro_valid_i & ~udma_shtdwn_i) begin
          ro_buf_ce_o     = 1'b1;
          next_state      = SAMPLE;
        end
      end
      SAMPLE: begin
        ro_buf_ce_o     = 1'b1;
        ro_vtransfer_o  = 1'b1;
        s_data          = ro_rdata_i;
        s_addr          = ro_raddr_i;
        next_state      = RUN;
      end
      RUN: begin
        udma_valid_o  = 1'b1;
        if (udma_ready_i) begin
          if (ro_valid_i & ~udma_shtdwn_i) begin
            ro_buf_ce_o     = 1'b1;
            next_state      = SAMPLE;
          end
          else
            next_state = IDLE;
        end
      end
    endcase
  end

  // state update
  always_ff @(posedge clk_i, negedge rstn_i) begin: update_state
    if(~rstn_i) begin
      state   <= IDLE;
      r_data  <= '0;
      r_addr  <= '0;
    end
    else begin
      state   <= next_state;
      r_data  <= s_data;
      r_addr  <= s_addr;
    end
  end

endmodule
