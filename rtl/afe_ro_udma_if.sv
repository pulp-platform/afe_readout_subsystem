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
//
// Author: Florian Glaser <glaserf@iis.ee.ethz.ch>

module afe_ro_udma_if 
#(
  parameter L2_DATA_WIDTH = 32,
  parameter L2_AWIDTH_NOAL = 12
) (
  input  logic  clk_i,
  input  logic  rst_ni,
  input  logic  test_mode_i,

  input  logic                      afero_valid_i,
  output logic                      afero_ready_o,
  output logic                      afero_buff_ce_o,
  input  logic  [L2_DATA_WIDTH-1:0] afero_wdata_i,
  input  logic [L2_AWIDTH_NOAL-1:0] afero_addr_i,
  input  logic                [1:0] afero_size_i,

  input  logic                      udma_shtdwn_i,

  input  logic                      udma_ready_i,
  output logic                      udma_valid_o,
  output logic  [L2_DATA_WIDTH-1:0] udma_data_o,
  output logic [L2_AWIDTH_NOAL-1:0] udma_addr_o,
  output logic                [1:0] udma_size_o
);

  enum logic {IDLE, PUSH} state_q, state_n;

  logic fifo_full, fifo_empty;
  logic fifo_push, fifo_pop;
  logic fifo_usage;

  logic afero_ready;


  assign afero_ready_o = afero_ready;
  assign udma_valid_o  = ~fifo_empty;

  assign fifo_pop = udma_valid_o & udma_ready_i;

  fifo_v3 #(
    .DATA_WIDTH ( L2_DATA_WIDTH + L2_AWIDTH_NOAL + 2 ),
    .DEPTH      ( 2 ) // buffer one transaction on each side
  )
  transaction_buffer_i (
    .clk_i,
    .rst_ni,
    .testmode_i ( test_mode_i ),

    .flush_i    ( 1'b0        ),

    .full_o     ( fifo_full   ),
    .empty_o    ( fifo_empty  ),
    .usage_o    ( fifo_usage  ),

    .data_i     ( {afero_size_i,afero_addr_i,afero_wdata_i} ),
    .data_o     ( {udma_size_o,udma_addr_o,udma_data_o}     ),

    .push_i     ( fifo_push   ),
    .pop_i      ( fifo_pop    )
  );

  always_comb begin
    fifo_push = 1'b0;

    afero_ready     = ~fifo_full & ~udma_shtdwn_i;
    afero_buff_ce_o = 1'b0;

    state_n   = state_q;

    case(state_q)
      IDLE: begin
        if (afero_valid_i & afero_ready) begin
          afero_buff_ce_o = 1'b1;
          state_n = PUSH;
        end
      end

      PUSH: begin
        fifo_push = 1'b1;
        /* modified ready condition, we have to commit to two pushes */
        afero_ready = ~fifo_full & ~udma_shtdwn_i & (~fifo_usage | fifo_pop);
        /* check if afero wants to push more and we can accept it */
        if (afero_valid_i & afero_ready) begin
          afero_buff_ce_o = 1'b1;
        end
        else begin
          state_n = IDLE;
        end
      end
    endcase
  end

  // state update
  always_ff @(posedge clk_i, negedge rst_ni) begin : update_state
    if(~rst_ni) begin
      state_q <= IDLE;
    end
    else begin
      state_q <= state_n;
    end
  end

endmodule
