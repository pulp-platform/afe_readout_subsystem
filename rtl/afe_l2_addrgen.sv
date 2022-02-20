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
// Adapted and extended from the udma addrgen by Antonio Pullini
// Authors: Yasmine Bennani <ybennani@student.ethz.ch>, Florian Glaser <glaserf@iis.ee.ethz.ch>

module afe_l2_addrgen #( 
  parameter AWIDTH     = 18,
  parameter TRANS_SIZE = 16
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,
  input  logic                  test_mode_i,

  input  logic     [AWIDTH-1:0] cfg_startaddr_i,
  input  logic [TRANS_SIZE-1:0] cfg_size_i,
  input  logic            [1:0] cfg_datasize_i,
  input  logic                  cfg_continuous_i,
  input  logic                  cfg_en_i,
  input  logic                  cfg_clr_i,
  output logic     [AWIDTH-1:0] cfg_curr_addr_o,
  output logic [TRANS_SIZE-1:0] cfg_wr_ptr_o,
  output logic [TRANS_SIZE-1:0] cfg_bytes_left_o,
  output logic                  cfg_en_o,
 
  /* asserted if half or full buffer filled */
  output logic                  event_o,

  input  logic                  transfer_valid_i
  );             

  /* registers */
  logic     [AWIDTH-1:0] address_q, address_n;
  logic [TRANS_SIZE-1:0] cnt_q, cnt_n;

  logic en_q, en_n;
  logic event_q, event_n;

  /* local signals */
  logic [TRANS_SIZE-1:0] datasize_incr;
  logic                  last_transfer;
    
  /* assignments */
  assign cfg_en_o         = en_q;
  assign event_o          = event_q;
  assign cfg_curr_addr_o  = address_q;
  assign cfg_bytes_left_o = cnt_q;
  assign cfg_wr_ptr_o     = address_q - cfg_startaddr_i;

  assign last_transfer    = (cnt_q <= datasize_incr);  

  /* address and counter incr/decr value in bytes */
  always_comb begin
    case (cfg_datasize_i)
      2'b00:   datasize_incr = 'd1;
      2'b01:   datasize_incr = 'd2;
      2'b10:   datasize_incr = 'd4;
      default: datasize_incr = 'd4;
    endcase
  end

  always_comb begin
    cnt_n     = cnt_q;
    address_n = address_q;
    en_n      = en_q;
    event_n   = 1'b0;

    if(cfg_en_i & ~en_q) begin
      cnt_n     = cfg_size_i;
      address_n = cfg_startaddr_i;
      en_n      = 1'b1;
    end
    else if (cfg_clr_i) begin
      cnt_n     =  '0;
      address_n =  '0;
      en_n      = 1'b0;
    end
    else if (en_q) begin
      event_n = transfer_valid_i && (last_transfer || (cnt_q == ((cfg_size_i>>1) + datasize_incr)));

      if (last_transfer & transfer_valid_i) begin
        if (~cfg_continuous_i & ~cfg_en_i) begin
          /* stop the channel */
          cnt_n     =  '0;
          address_n =  '0;
          en_n      = 1'b0;
        end
        else begin
          /* reload config */
          cnt_n     = cfg_size_i;
          address_n = cfg_startaddr_i;
          en_n      = 1'b1;
        end
      end
      else if (transfer_valid_i) begin
        /* regular transfer, update counter and address*/
        cnt_n     = cnt_q      - datasize_incr;
        address_n = address_q  + datasize_incr;
      end
    end
  end

  always_ff @(posedge clk_i, negedge rst_ni) begin
    if(~rst_ni) begin
      address_q <=  '0;
      cnt_q     <=  '0;
      en_q      <= 1'b0;
      event_q   <= 1'b0;
    end 
    else begin
      event_q   <= event_n;
      /* cg activation condition */
      if (cfg_en_i | cfg_clr_i | transfer_valid_i) begin
        address_q <= address_n;
        cnt_q     <= cnt_n;
        en_q      <= en_n;
      end
    end
  end

endmodule
