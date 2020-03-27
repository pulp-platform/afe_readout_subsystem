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

// register map
`define REG_UDMA_CFG    8'b00000000 // BASEADDR+0x00

module afe_ro_conf (
  input  logic                      clk_i,
  input  logic                      rstn_i,

  input  logic               [31:0] cfg_data_i,
  input  logic               [10:0] cfg_addr_i,
  input  logic                      cfg_valid_i,
  input  logic                      cfg_rwn_i,
  output logic               [31:0] cfg_data_o,
  output logic                      cfg_ready_o,

  output logic                      cfg_udma_shtdwn_o
);

  logic  r_udma_shtdwn;

  assign cfg_udma_shtdwn_o  = r_udma_shtdwn;

  always_ff @(posedge clk_i, negedge rstn_i) begin
    if(~rstn_i) begin
      r_udma_shtdwn <= 1'b0;
    end 
    else begin
      if (cfg_valid_i & ~cfg_rwn_i) begin
        case (cfg_addr_i[7:0])
          `REG_UDMA_CFG:
            r_udma_shtdwn <= cfg_data_i[0];
        endcase
      end
    end
  end

  always_comb begin
    cfg_data_o = '0;

    if (cfg_valid_i & cfg_rwn_i) begin
      case (cfg_addr_i[7:0])
        `REG_UDMA_CFG:
          cfg_data_o[0]  = r_udma_shtdwn;
      endcase
    end
  end

  assign cfg_ready_o = 1'b1;

endmodule 
