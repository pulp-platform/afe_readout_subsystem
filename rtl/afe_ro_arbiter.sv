// Copyright 2022 ETH Zurich and University of Bologna.
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


module afe_ro_arbiter #(
  parameter NUM_AFE     = 4,
  parameter DATA_WIDTH  = 32,
  parameter BUFF_AWIDTH = 10,
  parameter L2_AWIDTH   = 18
) (
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   test_mode_i,

  output logic                   buff_cen_o,
  output logic                   buff_rwn_o,
  output logic [BUFF_AWIDTH-1:0] buff_addr_o,
  output logic  [DATA_WIDTH-1:0] buff_wdata_o,

  output logic     [NUM_AFE-1:0] buff_rvalid_o,

  output logic                   intf_valid_o,
  input  logic                   intf_ready_i,
  output logic  [DATA_WIDTH-1:0] intf_wdata_o,
  output logic   [L2_AWIDTH-1:0] intf_addr_o,
  output logic             [1:0] intf_size_o,

  input  logic     [NUM_AFE-1:0] afe_valid_i,
  output logic     [NUM_AFE-1:0] wr_ready_o,

  output logic     [NUM_AFE-1:0] rd_ready_o,
  input  logic     [NUM_AFE-1:0] rd_valid_i,

  input  logic [NUM_AFE-1:0] [DATA_WIDTH-1:0] afe_data_i,

  input  logic [NUM_AFE-1:0] [DATA_WIDTH-1:0] l2_wdata_i,
  input  logic [NUM_AFE-1:0]  [L2_AWIDTH-1:0] l2_addr_i,
  input  logic [NUM_AFE-1:0]            [1:0] l2_size_i,

  input  logic [NUM_AFE-1:0][BUFF_AWIDTH-1:0] buff_wr_addr_i,
  input  logic [NUM_AFE-1:0][BUFF_AWIDTH-1:0] buff_rd_addr_i
);

  localparam NUM_AFE_LOG = $clog2(NUM_AFE);

  logic     [NUM_AFE-1:0] wr_grant, rd_grant, rd_grant_del;
  logic [NUM_AFE_LOG-1:0] wr_grant_bin, rd_grant_bin, rd_grant_del_bin;
  logic [BUFF_AWIDTH-1:0] wr_addr_sel, rd_addr_sel;

  logic wr_grant_any, rd_grant_any;
  logic rd_trnsct, rd_trnsct_del;


  /* address muxing */
  assign wr_addr_sel   = buff_wr_addr_i[wr_grant_bin];
  assign rd_addr_sel   = buff_rd_addr_i[rd_grant_bin];

  assign rd_trnsct     = ~wr_grant_any & rd_grant_any & intf_ready_i;
  
  assign buff_addr_o   = wr_grant_any ? wr_addr_sel : rd_addr_sel;
  assign buff_wdata_o  = afe_data_i[wr_grant_bin];
  assign buff_rwn_o    = ~wr_grant_any;
  assign buff_cen_o    = ~(rd_trnsct | wr_grant_any);

  assign buff_rvalid_o = rd_trnsct_del ? rd_grant_del : '0;

  assign intf_wdata_o  = rd_trnsct_del ? l2_wdata_i[rd_grant_del_bin] : '0;
  assign intf_addr_o   = rd_trnsct_del ? l2_addr_i[rd_grant_del_bin]  : '0;
  assign intf_size_o   = rd_trnsct_del ? l2_size_i[rd_grant_del_bin]  : 2'b00;

  assign intf_valid_o  = ~wr_grant_any & rd_grant_any;
  assign rd_ready_o    = rd_trnsct ? rd_grant : '0;

  assign wr_ready_o    = wr_grant_any ? wr_grant : '0;


  udma_arbiter #(
    .N ( NUM_AFE ),
    .S (       3 )
  )
  write_arbiter_i (
    .clk_i,
    .rstn_i      ( rst_ni ),

    .req_i       ( afe_valid_i  ),
    .grant_o     ( wr_grant     ),
    .grant_ack_i ( wr_grant_any ),
    .anyGrant_o  ( wr_grant_any )
  );

  onehot_to_bin #(
    .ONEHOT_WIDTH ( NUM_AFE )
  )
  grant_wr_bin_i (
    .onehot ( wr_grant     ),
    .bin    ( wr_grant_bin )
  );


  udma_arbiter #(
    .N ( NUM_AFE ),
    .S (       3 )
  )
  read_arbiter_i (
    .clk_i,
    .rstn_i      ( rst_ni ),

    .req_i       ( rd_valid_i   ),
    .grant_o     ( rd_grant     ),
    .grant_ack_i ( rd_trnsct    ),
    .anyGrant_o  ( rd_grant_any )
  );

  onehot_to_bin #(
    .ONEHOT_WIDTH ( NUM_AFE )
  )
  grant_rd_bin_i (
    .onehot ( rd_grant     ),
    .bin    ( rd_grant_bin )
  );

  onehot_to_bin #(
    .ONEHOT_WIDTH ( NUM_AFE )
  )
  grant_rd_bin_del_i (
    .onehot ( rd_grant_del     ),
    .bin    ( rd_grant_del_bin )
  );

  always_ff @(posedge clk_i, negedge rst_ni) begin
    if(~rst_ni) begin
      rd_trnsct_del <= 1'b0;
      rd_grant_del  <= '0;
    end
    else begin
      rd_trnsct_del <= rd_trnsct;
      rd_grant_del  <= rd_grant;
    end
  end

endmodule
