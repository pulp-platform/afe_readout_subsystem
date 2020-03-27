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


// This package is only an example - adapt it to your needs!
package afe_parameters_pkg;

   // number of bytes (2^x) that a transfer can have
   parameter AFE0_RX_TRANS_SIZE = 14;
   parameter AFE1_RX_TRANS_SIZE = 14;
   parameter AFE2_RX_TRANS_SIZE = 14;
   parameter AFE3_RX_TRANS_SIZE = 14;

   parameter integer AFE_RX_TRANS_SIZE [NUM_AFE_RX-1:0] = {AFE3_RX_TRANS_SIZE, AFE2_RX_TRANS_SIZE, AFE1_RX_TRANS_SIZE, AFE0_RX_TRANS_SIZE};

   // Define how AFE channels are mapped to L2 channels, three types are available:
   // 0 - direct mapped, 1 L2 channel per AFE channel
   // 1 - direct mapped with a number of sub-channels per channel that share one L2 pointer, each sub-channel gets mapped to base_addr+subch_id*stride_size
   // 2 - programmable AFE channel id for each L2 channel (used when #L2-chs should be much smaller than #AFE-chs)
   parameter integer AFE_RX_TYPE [NUM_AFE_RX-1:0] = {0, 0, 1, 2};

   // total data width, including all ids and flag data
   // we usually have, from MSB to LSB (with zero-padding in between): ch_id | (subch_id) | flags | payload data
   parameter AFE0_RX_DATA_W = 32;
   parameter AFE1_RX_DATA_W = 32;
   parameter AFE2_RX_DATA_W = 32;
   parameter AFE3_RX_DATA_W = 18;

   parameter integer AFE_RX_DATA_W     [NUM_AFE_RX-1:0] = {AFE3_RX_DATA_W, AFE2_RX_DATA_W, AFE1_RX_DATA_W, AFE0_RX_DATA_W};
   parameter integer AFE_RX_DATA_OFFS  [NUM_AFE_RX-1:0] = {AFE2_RX_DATA_W+AFE1_RX_DATA_W+AFE0_RX_DATA_W, AFE1_RX_DATA_W+AFE0_RX_DATA_W, AFE0_RX_DATA_W, 0};
   parameter AFE_RX_DATA_W_TOT = AFE3_RX_DATA_W+AFE2_RX_DATA_W+AFE1_RX_DATA_W+AFE0_RX_DATA_W;

   // payload data widths
   parameter AFE0_PL_DATA_W = 24;
   parameter AFE1_PL_DATA_W = 16;
   parameter AFE2_PL_DATA_W = 24;
   parameter AFE3_PL_DATA_W = 16;

   parameter integer      AFE_PL_DATA_W  [NUM_AFE_RX-1:0] = {AFE3_PL_DATA_W, AFE2_PL_DATA_W, AFE1_PL_DATA_W, AFE0_PL_DATA_W};

   parameter logic [31:0] AFE_RX_MASK_PL [NUM_AFE_RX-1:0] = {32'h0000FFFF, 32'h00FFFFFF, 32'h0000FFFF, 32'h00FFFFFF};
   parameter logic [31:0] AFE_RX_MASK_FL [NUM_AFE_RX-1:0] = {32'h0003FFFF, 32'h0FFFFFFF, 32'h0007FFFF, 32'h03FFFFFF};

   // AFE and L2 channel count definitions
   parameter AFE0_RX_NUM_CH = 32;
   parameter AFE1_RX_NUM_CH = 16;
   parameter AFE2_RX_NUM_CH = 12;
   parameter AFE3_RX_NUM_CH = 3;

   parameter integer AFE_RX_NUM_CHS [NUM_AFE_RX-1:0] = {AFE3_RX_NUM_CH, AFE2_RX_NUM_CH, AFE1_RX_NUM_CH, AFE0_RX_NUM_CH};
   parameter integer AFE_RX_OFFS_CH [NUM_AFE_RX-1:0] = {AFE2_RX_NUM_CH+AFE1_RX_NUM_CH+AFE0_RX_NUM_CH, AFE1_RX_NUM_CH+AFE0_RX_NUM_CH, AFE0_RX_NUM_CH, 0};
   parameter AFE_RX_MAX_NUM_CH  = AFE0_RX_NUM_CH;
   parameter AFE_RX_NUM_CHS_TOT = AFE3_RX_NUM_CH+AFE2_RX_NUM_CH+AFE1_RX_NUM_CH+AFE0_RX_NUM_CH;

   // required for RX type 2, otherwise #AFE_ch = #L2_ch
   parameter AFE0_RX_NUM_L2CH  = 4;

   parameter integer AFE_RX_NUM_L2CHS [NUM_AFE_RX-1:0] = {AFE3_RX_NUM_CH, AFE2_RX_NUM_CH, AFE1_RX_NUM_CH, AFE0_RX_NUM_L2CH};
   parameter integer AFE_RX_OFFS_L2CH [NUM_AFE_RX-1:0] = {AFE2_RX_NUM_CH+AFE1_RX_NUM_CH+AFE0_RX_NUM_L2CH, AFE1_RX_NUM_CH+AFE0_RX_NUM_L2CH, AFE0_RX_NUM_L2CH, 0};
   parameter AFE_RX_MAX_NUM_L2CH  =  AFE1_RX_NUM_CH;
   parameter AFE_RX_NUM_L2CH_TOT  =  AFE3_RX_NUM_CH+AFE2_RX_NUM_CH+AFE1_RX_NUM_CH+AFE0_RX_NUM_L2CH;

   // these parameters are only used for RX type 1
   parameter AFE_RX_STRIDE_SIZE = 12;
   parameter AFE1_RX_NUM_SUBCH  = 3;

   parameter integer AFE_RX_NUM_SUBCHS [NUM_AFE_RX-1:0] = {0, 0, AFE1_RX_NUM_SUBCH, 0};
   parameter AFE_RX_MAX_NUM_SUBCH = AFE1_RX_NUM_SUBCH;

   // channel-id bitfields definitions
   parameter AFE0_RX_CHID_LSB   = 26;
   parameter AFE0_RX_CHID_WIDTH = 5;
   parameter AFE1_RX_CHID_LSB   = 28;
   parameter AFE1_RX_CHID_WIDTH = 4;
   parameter AFE2_RX_CHID_LSB   = 28;
   parameter AFE2_RX_CHID_WIDTH = 4;
   parameter AFE3_RX_CHID_LSB   = 16;
   parameter AFE3_RX_CHID_WIDTH = 2;

   parameter AFE1_RX_SUBCHID_LSB   = 26;
   parameter AFE1_RX_SUBCHID_WIDTH = 2;

   parameter integer AFE_RX_CHID_LSB      [NUM_AFE_RX-1:0] = {AFE3_RX_CHID_LSB,   AFE2_RX_CHID_LSB,   AFE1_RX_CHID_LSB,   AFE0_RX_CHID_LSB};
   parameter integer AFE_RX_CHID_WIDTH    [NUM_AFE_RX-1:0] = {AFE3_RX_CHID_WIDTH, AFE2_RX_CHID_WIDTH, AFE1_RX_CHID_WIDTH, AFE0_RX_CHID_WIDTH};

   // for non-type-1 AFEs, the chid-LSB position must be used (for meta-data masking modes to work properly)
   parameter integer AFE_RX_SUBCHID_LSB   [NUM_AFE_RX-1:0] = {AFE3_RX_CHID_LSB,   AFE2_RX_CHID_LSB,   AFE1_RX_SUBCHID_LSB,   AFE0_RX_CHID_LSB};
   parameter integer AFE_RX_SUBCHID_WIDTH [NUM_AFE_RX-1:0] = {0,                  0,                  AFE1_RX_SUBCHID_WIDTH, 0};


   // flag-related definitions
   // controls whether or not flag-capturing is enabled for each AFE
   parameter integer AFE_FLAG_MASK [NUM_AFE-1:0] = {0, 1, 1, 1};

   parameter AFE0_RX_FLAG_LSB   = 25;
   parameter AFE0_RX_FLAG_WIDTH = 1;
   parameter AFE1_RX_FLAG_LSB   = 16;
   parameter AFE1_RX_FLAG_WIDTH = 3;
   parameter AFE2_RX_FLAG_LSB   = 26;
   parameter AFE2_RX_FLAG_WIDTH = 2;
   parameter AFE3_RX_FLAG_LSB   = 0;
   parameter AFE3_RX_FLAG_WIDTH = 0;

   parameter integer AFE_RX_FLAG_LSB   [NUM_AFE_RX-1:0] = {AFE3_RX_FLAG_LSB,   AFE2_RX_FLAG_LSB,   AFE1_RX_FLAG_LSB,   AFE0_RX_FLAG_LSB};
   parameter integer AFE_RX_FLAG_WIDTH [NUM_AFE_RX-1:0] = {AFE3_RX_FLAG_WIDTH, AFE2_RX_FLAG_WIDTH, AFE1_RX_FLAG_WIDTH, AFE0_RX_FLAG_WIDTH};

   parameter AFE_RX_MAX_FLAG_WIDTH = AFE1_RX_FLAG_WIDTH;

endpackage
