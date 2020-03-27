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

module afe_l2_addr_gen_type1 
#( 
  parameter AWIDTH              = 18,
  parameter TRANS_SIZE          = 16,
  parameter ADC_SUBCH_ID_WIDTH  = 2
) (
  input  logic  clk_i,
  input  logic  rstn_i,       

  input  logic             [AWIDTH-1:0] cfg_startaddr_i,     
  input  logic         [TRANS_SIZE-1:0] cfg_size_i,
  input  logic [ADC_SUBCH_ID_WIDTH-1:0] cfg_adc_subch_id_i,
  input  logic                          cfg_continuous_i,
  input  logic                          cfg_en_i,
  input  logic                          cfg_clr_i,
  output logic             [AWIDTH-1:0] cfg_curr_addr_o,
  output logic         [TRANS_SIZE-1:0] cfg_wr_ptr_o,
  output logic         [TRANS_SIZE-1:0] cfg_bytes_left_o,
  output logic                          cfg_en_o,
 
  output logic                          ch_event_o,          // sends event if half of the channel l2 memory area is full

  input  logic [ADC_SUBCH_ID_WIDTH-1:0] udma_subch_id_i,
  input  logic                          udma_vtransfer_i    // the ptrs should be updated only if a valid transfer has occured on the udma side
);


  localparam ADC_DATA_SIZE = 4;     // in bytes                  

  logic         [AWIDTH-1:0] r_addresses;
  logic     [TRANS_SIZE-1:0] r_counters;
  logic                      r_en;
  logic                      r_event;

  logic         [AWIDTH-1:0] s_addresses;
  logic     [TRANS_SIZE-1:0] s_counters;
  logic                      s_en;
  logic                      s_event;

  logic                      s_last_transfer;
    
  /* Outputs assignments */
  assign cfg_en_o         = r_en;
  assign ch_event_o       = r_event;
  assign cfg_curr_addr_o  = r_addresses;
  assign cfg_bytes_left_o = r_counters;
  assign cfg_wr_ptr_o     = r_addresses - cfg_startaddr_i;

  assign s_last_transfer  = (r_counters <= ADC_DATA_SIZE);  


  always_comb begin : proc_next_val
    s_counters  = r_counters;  
    s_addresses = r_addresses;  
    s_en        = r_en;  
    s_event     = 1'b0;

    if(cfg_en_i && !r_en) begin //store config data when enabling the channel
        s_counters  =  cfg_size_i;
        s_addresses =  cfg_startaddr_i;
        s_en        =  1'b1;
        s_event     =  1'b0;
    end
    else if (cfg_clr_i) begin
        s_counters  =   '0;
        s_addresses =   '0;
        s_en        =  1'b0;
        s_event     =  1'b0;
    end
    else begin
      if (r_en) begin //if channel enabled then
        //s_event   = (r_counters <= (cfg_size_i>>1));                 // raises event if counters <= half of the channel memory size
        s_event   = udma_vtransfer_i && (udma_subch_id_i == cfg_adc_subch_id_i) && ((r_counters == ((cfg_size_i>>1) + ADC_DATA_SIZE)) || s_last_transfer);

        if (s_last_transfer && udma_vtransfer_i && (udma_subch_id_i == cfg_adc_subch_id_i)) begin
          if (!cfg_continuous_i && !cfg_en_i) begin
            s_en    = 1'b0;                                      //if not in continuous mode then stop the channel
            s_counters  = '0;
            s_addresses = '0;
          end
          else begin
            s_counters  = cfg_size_i;                            //reload the buffer size
            s_addresses = cfg_startaddr_i;                       //reload the start address
            s_en        = 1'b1;
          end
        end
        else if (udma_vtransfer_i && (udma_subch_id_i == cfg_adc_subch_id_i)) begin // not last transfer and valid transfer has occured
          s_counters  = r_counters - ADC_DATA_SIZE;                //decrement the remaining bytes of the channel
          s_addresses = r_addresses + ADC_DATA_SIZE;               //increment the address
        end
      end
      else begin // if channel not enabled
        s_event     =  1'b0;
      end
    end
  end    


  always_ff @(posedge clk_i, negedge rstn_i) begin
    if(~rstn_i) begin
      r_addresses  <=  '0;
      r_counters   <=  '0;
      r_en         <= 1'b0;
      r_event      <= 1'b0;
    end 
    else begin
      r_counters   <= s_counters;
      r_addresses  <= s_addresses;
      r_en         <= s_en;
      r_event      <= s_event;
    end
  end

  
endmodule