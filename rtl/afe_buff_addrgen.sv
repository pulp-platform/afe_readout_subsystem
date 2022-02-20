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

module afe_buff_addrgen #(
  parameter AWIDTH         = 10,
  parameter TRANS_SIZE     = 16
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,
  input  logic                  test_mode_i,
  
  input  logic     [AWIDTH-1:0] cfg_startaddr_i,
  input  logic [TRANS_SIZE-1:0] cfg_size_i,
  input  logic                  cfg_continuous_i,
  input  logic                  cfg_overflow_i,
  input  logic [TRANS_SIZE-1:0] cfg_flevel_i,
  input  logic                  cfg_en_i,
  input  logic                  cfg_clr_i,
  output logic [TRANS_SIZE-1:0] cfg_smpls_left_o,
  output logic                  cfg_en_o,

  output logic     [AWIDTH-1:0] wr_addr_o,
  output logic     [AWIDTH-1:0] rd_addr_o,

  /* from buffer arbiter, signals granted transactions */
  input  logic                  wr_ready_i,
  input  logic                  rd_ready_i,

  /* to buffer arbiter, signals non-empty buffer */
  output logic                  rd_valid_o,

  output logic                  flevel_event_o
);

  /* address increment and decrement step */
  localparam ADDR_INC = 1;

  /* read and write pointers */
  logic     [AWIDTH-1:0] rd_ptr_q, rd_ptr_n;
  logic     [AWIDTH-1:0] wr_ptr_q, wr_ptr_n;

  /* transfer counter: initialized to transfer size, decr on write, incr on read */
  logic [TRANS_SIZE-1:0] cnt_q, cnt_n;

  /* internal control registers */
  logic en_q, en_n;
  logic flevel_event_q, flevel_event_n;
  
  /* internal signals */
  logic trigger_event;

  /* output assignments */
  assign cfg_smpls_left_o = cnt_q;
  assign cfg_en_o         = en_q;

  assign wr_addr_o        = wr_ptr_q;
  assign rd_addr_o        = rd_ptr_q;
  assign rd_valid_o       = en_q ? ~(cnt_q == cfg_size_i) : 1'b0;

  assign flevel_event_o   = flevel_event_q;

  /* generation of fill-level event */
  assign flevel_event_n = en_q ? (cnt_q == (cfg_size_i - cfg_flevel_i)) & trigger_event : 1'b0;


  always_comb begin
    rd_ptr_n       = rd_ptr_q;
    wr_ptr_n       = wr_ptr_q;
    cnt_n          = cnt_q;
    en_n           = en_q;

    trigger_event  = 1'b0;

    if (cfg_en_i & ~en_q) begin
      /* generator was disabled, new config is pushed */
      wr_ptr_n  = cfg_startaddr_i;
      rd_ptr_n  = cfg_startaddr_i;
      cnt_n     = cfg_size_i;
      en_n      = 1'b1;
    end
    else if (cfg_clr_i) begin
      /* clear config and halt */
      wr_ptr_n  = '0;
      rd_ptr_n  = '0;
      cnt_n     = '0;
      en_n      = 1'b0;
    end
    else if (en_q) begin
      if (rd_ready_i) begin
        /* read transfer occurred, increase read ptr */
        rd_ptr_n  = wrap_around(rd_ptr_q);
        cnt_n     = cnt_q + ADDR_INC;
      end

      if (wr_ready_i) begin
        /* write transfer occurred, manage write ptr */
        if (cnt_q > ADDR_INC) begin /* still room in buffer */
          wr_ptr_n      = wrap_around(wr_ptr_q);
          trigger_event = 1'b1;
          /* only change fill state if no read occurs */
          if (rd_ready_i)
            cnt_n = cnt_q;
          else
            cnt_n = cnt_q - ADDR_INC;
        end
        else if (cnt_q == '0) begin
          /* overflow occurred, overwrite old samples */
          wr_ptr_n  = wrap_around(wr_ptr_q);
          rd_ptr_n  = wrap_around(rd_ptr_q);
          cnt_n     = '0;
        end
        else begin /* last write transfer.. */
          if (rd_ready_i) begin /* ..but read also happened */
            wr_ptr_n      = wrap_around(wr_ptr_q);
            cnt_n         = cnt_q;
            trigger_event = 1'b1;
          end
          else if (cfg_overflow_i) begin
            /* prepare for overwriting old samples */
            wr_ptr_n  = wrap_around(wr_ptr_q);
            cnt_n     = 0;
          end
          else if (~cfg_continuous_i & ~cfg_en_i) begin /* no further transfer */
            /* clear config and halt */
            wr_ptr_n  = '0;
            rd_ptr_n  = '0;
            cnt_n     = '0;
            en_n      = 1'b0;
          end
          else begin /* continuous mode, reload config */
            wr_ptr_n  = cfg_startaddr_i;
            rd_ptr_n  = cfg_startaddr_i;
            cnt_n     = cfg_size_i;
            en_n      = 1'b1;
          end
        end
      end
    end
  end

  always_ff @(posedge clk_i, negedge rst_ni) begin 
    if(~rst_ni) begin
      rd_ptr_q        <= '0;
      wr_ptr_q        <= '0;
      cnt_q           <= '0;
      en_q            <= 1'b0;
      flevel_event_q  <= 1'b0;
    end 
    else begin
      flevel_event_q  <= flevel_event_n;
      /* cg activation condition */
      if (cfg_en_i | cfg_clr_i | rd_ready_i | wr_ready_i) begin
        rd_ptr_q        <= rd_ptr_n;
        wr_ptr_q        <= wr_ptr_n;
        cnt_q           <= cnt_n;
        en_q            <= en_n;
      end
    end
  end

  function [AWIDTH-1:0] wrap_around;
    input [AWIDTH-1:0] ptr;
    begin
      if (ptr == (cfg_startaddr_i + cfg_size_i - ADDR_INC))
        return cfg_startaddr_i;
      else
        return ptr + ADDR_INC;
    end
  endfunction

endmodule
