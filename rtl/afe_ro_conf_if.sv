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
// Authors: Florian Glaser <glaserf@iis.ee.ethz.ch>

// register map for internal registers
`define REG_UDMA_CFG    8'b00000000 // BASEADDR+0x00
`define REG_AFE_CG      8'b00000001 // BASEADDR+0x04

module afe_ro_conf_if #(
  parameter NUM_AFE        = 4,
  parameter APB_ADDR_WIDTH = 32
) (
  input  logic clk_i,
  input  logic rst_ni,
  input  logic test_mode_i,

  input  logic                      apb_sel_i,
  input  logic                      apb_en_i,
  input  logic                      apb_write_i,
  input  logic [APB_ADDR_WIDTH-1:0] apb_address_i,
  input  logic               [31:0] apb_wdata_i,
  output logic                      apb_ready_o,
  output logic                      apb_slverr_o,
  output logic               [31:0] apb_rdata_o,

  output logic               [10:0] cfg_addr_o,
  output logic               [31:0] cfg_wdata_o,
  output logic  [NUM_AFE-1:0]       cfg_valid_o,
  input  logic  [NUM_AFE-1:0]       cfg_ready_i,
  output logic                      cfg_rwn_o,
  input  logic  [NUM_AFE-1:0][31:0] cfg_rdata_i,

  output logic                      udma_shtdwn_o,
  output logic  [NUM_AFE-1:0]       afe_top_clk_o
);

  /* top-level module decoder/routing */
  logic             apb_valid;
  logic [2:0]       afe_sel;
  logic [7:0]       cfg_ready;
  logic [7:0][31:0] cfg_rdata;

  logic        cfg_valid_int;
  logic        cfg_ready_int;
  logic [31:0] cfg_rdata_int;

  assign afe_sel = apb_address_i[12:10];

  assign cfg_ready[7]           = cfg_ready_int;
  assign cfg_ready[6:NUM_AFE]   = '1;
  assign cfg_ready[NUM_AFE-1:0] = cfg_ready_i;

  assign cfg_rdata[7]           = cfg_rdata_int;
  assign cfg_rdata[6:NUM_AFE]   = '0;
  assign cfg_rdata[NUM_AFE-1:0] = cfg_rdata_i;

  assign apb_valid    = apb_en_i & apb_sel_i;
  assign apb_slverr_o = 1'b0;
  assign apb_ready_o  = apb_valid ? cfg_ready[afe_sel] : 1'b0;
  assign apb_rdata_o  = apb_valid ? cfg_rdata[afe_sel] : 1'b0;

  assign cfg_addr_o   = apb_address_i[12:2];
  assign cfg_wdata_o  = apb_wdata_i;
  assign cfg_rwn_o    = ~apb_write_i;

  always_comb begin
    cfg_valid_o   = '0;
    cfg_valid_int = 1'b0;

    if (apb_valid) begin
      if (afe_sel == 3'b111)
        cfg_valid_int = 1'b1;
      else
        cfg_valid_o[afe_sel] = 1'b1;
    end
  end

  /* internal registers */
  logic               r_udma_shtdwn;
  logic [NUM_AFE-1:0] r_afe_cg;

  assign cfg_ready_int = 1'b1;

  assign udma_shtdwn_o = r_udma_shtdwn;

  /* write process */
  always_ff @(posedge clk_i, negedge rst_ni) begin
    if(~rst_ni) begin
      r_udma_shtdwn <= 1'b0;
      r_afe_cg      <= '0;
    end 
    else begin
      if (cfg_valid_int & ~cfg_rwn_o) begin
        case (cfg_addr_o[7:0])
          `REG_UDMA_CFG: r_udma_shtdwn <= cfg_wdata_o[0];
          `REG_AFE_CG:   r_afe_cg      <= cfg_wdata_o[NUM_AFE-1:0];
        endcase
      end
    end
  end

  /* read decoder */
  always_comb begin
    cfg_rdata_int = '0;

    if (cfg_valid_int & cfg_rwn_o) begin
      case (cfg_addr_o[7:0])
        `REG_UDMA_CFG: cfg_rdata_int[0]           = r_udma_shtdwn;
        `REG_AFE_CG:   cfg_rdata_int[NUM_AFE-1:0] = r_afe_cg;
      endcase
    end
  end

  /* AFE top-level clock gates*/
  for (genvar I=0; I<NUM_AFE; I++) begin : AFE_TOP_CG
    pulp_clock_gating afe_top_cg_i
    (
      .clk_i     ( clk_i            ),
      .en_i      ( r_afe_cg[I]      ),
      .test_en_i ( test_mode_i      ),
      .clk_o     ( afe_top_clk_o[I] )
    );
  end

endmodule
