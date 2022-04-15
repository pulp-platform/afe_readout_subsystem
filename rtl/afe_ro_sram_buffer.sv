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
  parameter AFE_DATA_WIDTH = 32,
  parameter ADDR_WIDTH     = 10,
  parameter BIST_EN        = 0
)(
  input  logic clk_i,
  input  logic rst_ni,

  input  logic bist_en_i,
  output logic bist_done_o,
  output logic bist_fail_o,

  input  logic                      cen_i,    // active low chip enable
  input  logic                      wen_i,    // active low write enable
  input  logic [AFE_DATA_WIDTH-1:0] data_i,   
  input  logic     [ADDR_WIDTH-1:0] addr_i,   // read or write address
  output logic [AFE_DATA_WIDTH-1:0] data_o
);

  logic           [31:0] bwen;  // bit write enable
  logic           [31:0] rdata; // sram data output 
  logic           [31:0] wdata; // sram data input

  logic                  bist_wr_en, bist_rd_en, bist_cen;
  logic           [31:0] bist_wdata;
  logic [ADDR_WIDTH-1:0] bist_waddr;
  logic [ADDR_WIDTH-1:0] bist_raddr;
  logic [ADDR_WIDTH-1:0] bist_addr;

  assign data_o = rdata[AFE_DATA_WIDTH-1:0];
  assign wdata  = {{32-AFE_DATA_WIDTH{1'b0}}, data_i}; 
  assign bwen   = {{32-AFE_DATA_WIDTH{1'b1}}, {AFE_DATA_WIDTH{1'b0}}};

  assign bist_cen  = ~(bist_wr_en | bist_rd_en);
  assign bist_addr = bist_wr_en ? bist_waddr : bist_raddr;

  sram_wrapper_32b #(
    .ADDR_WIDTH ( ADDR_WIDTH )
  ) sram_i (
    .clk_i   ( clk_i ),
    .rdata_o ( rdata ),
    .wdata_i ( bist_en_i ? bist_wdata  : wdata  ),
    .ce_ni   ( bist_en_i ? bist_cen    : cen_i  ),
    .we_ni   ( bist_en_i ? ~bist_wr_en : wen_i  ),
    .bwe_ni  ( bwen  ),
    .addr_i  ( bist_en_i ? bist_addr   : addr_i )
  );


  if (BIST_EN) begin : BIST_GEN
    bist_wrapper #(
      .ADDR_WIDTH ( ADDR_WIDTH ),
      .DATA_WIDTH (         32 )
    )
    bist_wrapper_i
    (
      .clk_i,
      .rst_ni,
      
      .en_i      ( bist_en_i   ),
      .done_o    ( bist_done_o ),
      .fail_o    ( bist_fail_o ),
  
      .wr_en_o   ( bist_wr_en  ),
      .rd_en_o   ( bist_rd_en  ),
      .wr_data_o ( bist_wdata  ),
      .rd_data_i ( rdata       ),
      .wr_addr_o ( bist_waddr  ),
      .rd_addr_o ( bist_raddr  )
    );
  end
  else begin : BIST_NOGEN
    assign bist_wr_en = 1'b0;
    assign bist_rd_en = 1'b0;
    assign bist_wdata = '0;
    assign bist_waddr = '0;
    assign bist_raddr = '0;
  end

endmodule
