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

// Authors: Yasmine Bennani <ybennani@student.ethz.ch>, Florian Glaser <glaserf@iis.ee.ethz.ch>

module afe_buf_addr_gen #(
  parameter AWIDTH         = 10,
  parameter TRANS_SIZE     = 16
) (
  input  logic                      clk_i,
  input  logic                      rstn_i,             
  
  input  logic         [AWIDTH-1:0] cfg_startaddr_i,     
  input  logic     [TRANS_SIZE-1:0] cfg_size_i,
  input  logic                      cfg_continuous_i,
  input  logic     [TRANS_SIZE-1:0] cfg_flevel_i,
  input  logic                      cfg_en_i,
  input  logic                      cfg_clr_i,
  output logic         [AWIDTH-1:0] cfg_curr_addr_o,
  output logic     [TRANS_SIZE-1:0] cfg_bytes_left_o,
  output logic                      cfg_en_o,

  input  logic                      buf_rwn_i,           // read mode: 1, write mode: 0
  input  logic                      adc_grant_i,         // HS Pair: if the adc has been granted
  output logic                      write_grant_ack_o,   // HS Pair: grant ack if the ADC has been able to write the buffer when granted
  output logic                      buf_event_o,         // sends event if the filling level has been reached

  /* UDMA protocol IF signals */
  input  logic                      udma_vtransfer_i,    // a valid read transfer has occured on the UDMA side 
  output logic                      udma_read_valid_o    // the read pointer points to valid data
);


  /* the address increment */
  localparam ADDR_INC = 1;                       

  /* Read and Write pointers */
  logic [AWIDTH-1:0] s_rd_ptr, r_rd_ptr;
  logic [AWIDTH-1:0] s_wr_ptr, r_wr_ptr;  

  logic              s_en, r_en;

  logic              s_wr_grant_ack;
  logic              s_empty_event;
  logic              s_flevel_event;
  logic              r_flevel_event;
  logic              s_trig_event;

  logic              s_udma_read_valid;

  logic [TRANS_SIZE-1:0] s_counters, r_counters;         // Decreases when a write occurs, increases when a read occurs 


  /* Outputs assignments */
  assign cfg_bytes_left_o  = r_counters;
  assign cfg_en_o          = r_en;
  assign write_grant_ack_o = s_wr_grant_ack;
  assign buf_event_o       = r_flevel_event;
  assign udma_read_valid_o = s_udma_read_valid;

  always_comb begin
    if(~buf_rwn_i) begin
      cfg_curr_addr_o = r_wr_ptr;
    end 
    else if(udma_vtransfer_i) begin
      cfg_curr_addr_o = s_rd_ptr;
    end
    else begin 
      cfg_curr_addr_o = r_rd_ptr;
    end
  end

  /* Events assignments */
  assign s_empty_event  = (r_en) ? (r_counters == cfg_size_i)                                  : 1'b0; 
  assign s_flevel_event = (r_en) ? (r_counters == (cfg_size_i - cfg_flevel_i)) && s_trig_event : 1'b0;

  assign s_udma_read_valid = (r_en) ? ~s_empty_event : 1'b0;      // the read ptr points to valid data only if the buffer is not empty


  always_comb begin: proc_next_ptr
    s_wr_grant_ack = 1'b0;
    s_rd_ptr       = r_rd_ptr;
    s_wr_ptr       = r_wr_ptr;
    s_counters     = r_counters;
    s_trig_event   = 1'b0;
    s_en           = r_en;


    if(cfg_en_i && ~r_en) begin // store config data when enabling the channel
      s_wr_ptr       = cfg_startaddr_i;
      s_rd_ptr       = cfg_startaddr_i;
      s_counters     = cfg_size_i;
      s_wr_grant_ack = 1'b0; 
      s_en           = 1'b1;
    end 
    else if (cfg_clr_i) begin
      s_wr_ptr       = '0;
      s_rd_ptr       = '0;
      s_counters     = '0;
      s_wr_grant_ack = 1'b0;
      s_en           = 1'b0;
    end 
    else begin
      if( udma_vtransfer_i ) begin // if a read transfer has occured
      //if(adc_grant_i && buf_rwn_i) begin // a read transfer will occur
        s_wr_grant_ack = 1'b0;
        s_rd_ptr       = wrap_around(r_rd_ptr);
        s_wr_ptr       = r_wr_ptr;
        s_counters     = r_counters + ADDR_INC;
        s_trig_event   = 1'b0; // TODO check: This should prevent an event when we read below the threshold again
        s_en           = 1'b1;
      end 

      if ( adc_grant_i && r_en ) begin // if adc granted and enabled
        if(~buf_rwn_i) begin // if in write mode -- TODO check prob not needed
          if(r_counters <= ADDR_INC) begin // if the buffer is full (ie cannot write)
            if(udma_vtransfer_i) begin
              s_wr_ptr       = wrap_around(r_wr_ptr);                                   // a write can occur because a read has just occured
              s_counters     = r_counters - ADDR_INC;
              s_trig_event   = 1'b1;
              s_wr_grant_ack = 1'b1;                                                    // the ADC is able to write in the buffer
              s_en           = 1'b1;
            end
            else if(~cfg_continuous_i && ~cfg_en_i) begin // if not in continuous mode, stop the channel
              s_rd_ptr       = '0;
              s_wr_ptr       = '0;
              s_counters     = '0;
              s_wr_grant_ack = 1'b0;                                                    // the ADC cannot write in the channel
              s_en           = 1'b0;
            end else begin // if in continuous mode, reload config
              s_rd_ptr       = cfg_startaddr_i;                                       
              s_wr_ptr       = cfg_startaddr_i;
              s_counters     = cfg_size_i;
              s_wr_grant_ack = 1'b1;
              s_en           = 1'b1;
            end
          end 
          else begin // if buffer not full
            s_wr_grant_ack  = 1'b1;                                                        // if the buffer is not full, np to access it
            s_en            = 1'b1;
            s_wr_ptr        = wrap_around(r_wr_ptr);
            s_counters      = r_counters - ADDR_INC;
            s_trig_event    = 1'b1;
          end
        end 
      end 
    end
  end

  always_ff @(posedge clk_i, negedge rstn_i) begin 
    if(~rstn_i) begin
      r_rd_ptr        <= '0;
      r_wr_ptr        <= '0;
      r_counters      <= '0;
      r_en            <= 1'b0;
      r_flevel_event  <= 1'b0;
    end 
    else begin
      r_rd_ptr        <= s_rd_ptr;
      r_wr_ptr        <= s_wr_ptr;
      r_counters      <= s_counters;
      r_en            <= s_en;
      r_flevel_event  <= s_flevel_event;
    end
  end

  function [AWIDTH-1:0] wrap_around;
    input [AWIDTH-1:0] ptr;
    begin
      if(ptr == (cfg_startaddr_i + cfg_size_i - ADDR_INC)) // if the ptr has reached the end, continue from the beginning
        return cfg_startaddr_i;
      else
        return ptr + ADDR_INC;
    end
  endfunction

endmodule