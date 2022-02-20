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

module afe_ro_sram_buffer #(
  AFE_DATA_WIDTH = 32,
  ADDR_WIDTH     = 10
)(
  input  logic clk_i,

  input  logic                      cen_i,    // active low chip enable
  input  logic                      wen_i,    // active low write enable
  input  logic [AFE_DATA_WIDTH-1:0] data_i,   
  input  logic     [ADDR_WIDTH-1:0] addr_i,   // read or write address
  output logic [AFE_DATA_WIDTH-1:0] data_o
);

  logic [31:0] bwen;  // bit write enable
  logic [31:0] rdata; // sram data output 
  logic [31:0] wdata; // sram data input

  assign data_o = rdata[AFE_DATA_WIDTH-1:0];
  assign wdata  = {{32-AFE_DATA_WIDTH{1'b0}}, data_i}; 
  assign bwen   = {{32-AFE_DATA_WIDTH{1'b1}}, {AFE_DATA_WIDTH{1'b0}}};

  sram_wrapper_32b #(
    .ADDR_WIDTH ( ADDR_WIDTH )
  ) sram_i (
    .clk_i   ( clk_i  ),
    .rdata_o ( rdata  ),
    .wdata_i ( wdata  ),
    .ce_ni   ( cen_i  ),
    .we_ni   ( wen_i  ),
    .bwe_ni  ( bwen   ),
    .addr_i  ( addr_i )
  );

endmodule
