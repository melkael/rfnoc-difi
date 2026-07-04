//
// Copyright 2026 Northeastern University
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module: rfnoc_block_difi_deframer
//
// Description:
//
//   Inverse of rfnoc_block_difi_basic (TX direction). Expects CHDR data
//   packets whose payload starts with a 28-byte (7 x 32-bit word) DIFI
//   header followed by sc16 samples. Strips the DIFI header words from the
//   payload and shortens the CHDR length field by 28 bytes, so a plain
//   CHDR sample packet continues downstream (e.g., into a DUC and radio).
//
//   The DIFI header contents (stream ID, OUI, timestamps, ...) are
//   discarded; CHDR context words other than the header (e.g., a CHDR
//   timestamp) pass through unmodified. Packets must contain at least one
//   sample after the DIFI header (i.e., payload of at least 8 words).
//
//   Sample count alignment (DIFI compliance): DIFI permits any sample
//   count per packet, but stock UHD DSP blocks such as the DUC ignore
//   tkeep on their input (rfnoc_block_duc.v ties it off), so packets
//   ending in a partial CHDR word would pick up a phantom pad sample.
//   To accept arbitrary DIFI packets while emitting only aligned ones,
//   this block re-packs across packet boundaries: when a packet's
//   available sample count (carried residue + new samples) is odd, the
//   final sample is retained and prepended to the next packet. A packet
//   whose CHDR header has EOB set flushes the residue (its output packet
//   may then be odd; it is the last of the burst, so the pad sample the
//   DUC appends is inconsequential). Packets that would emit zero
//   samples (single sample, no residue, no EOB) are dropped whole and
//   their sample carried forward. The sample *stream* is preserved
//   bit-exactly; only packet boundaries move.
//
//   Limitations: CHDR timestamps pass through unmodified, so timed
//   streams are only sample-accurate when every packet is already
//   aligned (untimed streaming recommended for odd counts). Each packet
//   must contain at least one sample after the DIFI header.
//
// Parameters:
//
//   THIS_PORTID : Control crossbar port to which this block is connected
//   CHDR_W      : AXIS-CHDR data bus width
//   MTU         : Maximum transmission unit (i.e., maximum packet size in
//                 CHDR words is 2**MTU).
//

`default_nettype none


module rfnoc_block_difi_deframer #(
  parameter [9:0] THIS_PORTID     = 10'd0,
  parameter       CHDR_W          = 64,
  parameter [5:0] MTU             = 10
)(
  // RFNoC Framework Clocks and Resets
  input  wire                   rfnoc_chdr_clk,
  input  wire                   rfnoc_ctrl_clk,
  // RFNoC Backend Interface
  input  wire [511:0]           rfnoc_core_config,
  output wire [511:0]           rfnoc_core_status,
  // AXIS-CHDR Input Ports (from framework)
  input  wire [(1)*CHDR_W-1:0] s_rfnoc_chdr_tdata,
  input  wire [(1)-1:0]        s_rfnoc_chdr_tlast,
  input  wire [(1)-1:0]        s_rfnoc_chdr_tvalid,
  output wire [(1)-1:0]        s_rfnoc_chdr_tready,
  // AXIS-CHDR Output Ports (to framework)
  output wire [(1)*CHDR_W-1:0] m_rfnoc_chdr_tdata,
  output wire [(1)-1:0]        m_rfnoc_chdr_tlast,
  output wire [(1)-1:0]        m_rfnoc_chdr_tvalid,
  input  wire [(1)-1:0]        m_rfnoc_chdr_tready,
  // AXIS-Ctrl Input Port (from framework)
  input  wire [31:0]            s_rfnoc_ctrl_tdata,
  input  wire                   s_rfnoc_ctrl_tlast,
  input  wire                   s_rfnoc_ctrl_tvalid,
  output wire                   s_rfnoc_ctrl_tready,
  // AXIS-Ctrl Output Port (to framework)
  output wire [31:0]            m_rfnoc_ctrl_tdata,
  output wire                   m_rfnoc_ctrl_tlast,
  output wire                   m_rfnoc_ctrl_tvalid,
  input  wire                   m_rfnoc_ctrl_tready
);

  //---------------------------------------------------------------------------
  // Signal Declarations
  //---------------------------------------------------------------------------

  // Clocks and Resets
  wire               ctrlport_clk;
  wire               ctrlport_rst;
  wire               axis_data_clk;
  wire               axis_data_rst;
  // CtrlPort Master
  wire               m_ctrlport_req_wr;
  wire               m_ctrlport_req_rd;
  wire [19:0]        m_ctrlport_req_addr;
  wire [31:0]        m_ctrlport_req_data;
  wire               m_ctrlport_resp_ack;
  wire [31:0]        m_ctrlport_resp_data;
  // Payload Stream to User Logic: in
  wire [32*1-1:0]    m_in_payload_tdata;
  wire [1-1:0]       m_in_payload_tkeep;
  wire               m_in_payload_tlast;
  wire               m_in_payload_tvalid;
  wire               m_in_payload_tready;
  // Context Stream to User Logic: in
  wire [CHDR_W-1:0]  m_in_context_tdata;
  wire [3:0]         m_in_context_tuser;
  wire               m_in_context_tlast;
  wire               m_in_context_tvalid;
  wire               m_in_context_tready;
  // Payload Stream from User Logic: out
  wire [32*1-1:0]    s_out_payload_tdata;
  wire [0:0]         s_out_payload_tkeep;
  wire               s_out_payload_tlast;
  wire               s_out_payload_tvalid;
  wire               s_out_payload_tready;
  // Context Stream from User Logic: out
  wire [CHDR_W-1:0]  s_out_context_tdata;
  wire [3:0]         s_out_context_tuser;
  wire               s_out_context_tlast;
  wire               s_out_context_tvalid;
  wire               s_out_context_tready;

  //---------------------------------------------------------------------------
  // NoC Shell
  //---------------------------------------------------------------------------

  noc_shell_difi_deframer #(
    .CHDR_W              (CHDR_W),
    .THIS_PORTID         (THIS_PORTID),
    .MTU                 (MTU)
  ) noc_shell_difi_deframer_i (
    //---------------------
    // Framework Interface
    //---------------------

    // Clock Inputs
    .rfnoc_chdr_clk      (rfnoc_chdr_clk),
    .rfnoc_ctrl_clk      (rfnoc_ctrl_clk),
    // Reset Outputs
    .rfnoc_chdr_rst      (),
    .rfnoc_ctrl_rst      (),
    // RFNoC Backend Interface
    .rfnoc_core_config   (rfnoc_core_config),
    .rfnoc_core_status   (rfnoc_core_status),
    // CHDR Input Ports  (from framework)
    .s_rfnoc_chdr_tdata  (s_rfnoc_chdr_tdata),
    .s_rfnoc_chdr_tlast  (s_rfnoc_chdr_tlast),
    .s_rfnoc_chdr_tvalid (s_rfnoc_chdr_tvalid),
    .s_rfnoc_chdr_tready (s_rfnoc_chdr_tready),
    // CHDR Output Ports (to framework)
    .m_rfnoc_chdr_tdata  (m_rfnoc_chdr_tdata),
    .m_rfnoc_chdr_tlast  (m_rfnoc_chdr_tlast),
    .m_rfnoc_chdr_tvalid (m_rfnoc_chdr_tvalid),
    .m_rfnoc_chdr_tready (m_rfnoc_chdr_tready),
    // AXIS-Ctrl Input Port (from framework)
    .s_rfnoc_ctrl_tdata  (s_rfnoc_ctrl_tdata),
    .s_rfnoc_ctrl_tlast  (s_rfnoc_ctrl_tlast),
    .s_rfnoc_ctrl_tvalid (s_rfnoc_ctrl_tvalid),
    .s_rfnoc_ctrl_tready (s_rfnoc_ctrl_tready),
    // AXIS-Ctrl Output Port (to framework)
    .m_rfnoc_ctrl_tdata  (m_rfnoc_ctrl_tdata),
    .m_rfnoc_ctrl_tlast  (m_rfnoc_ctrl_tlast),
    .m_rfnoc_ctrl_tvalid (m_rfnoc_ctrl_tvalid),
    .m_rfnoc_ctrl_tready (m_rfnoc_ctrl_tready),

    //---------------------
    // Client Interface
    //---------------------

    // CtrlPort Clock and Reset
    .ctrlport_clk              (ctrlport_clk),
    .ctrlport_rst              (ctrlport_rst),
    // CtrlPort Master
    .m_ctrlport_req_wr         (m_ctrlport_req_wr),
    .m_ctrlport_req_rd         (m_ctrlport_req_rd),
    .m_ctrlport_req_addr       (m_ctrlport_req_addr),
    .m_ctrlport_req_data       (m_ctrlport_req_data),
    .m_ctrlport_resp_ack       (m_ctrlport_resp_ack),
    .m_ctrlport_resp_data      (m_ctrlport_resp_data),

    // AXI-Stream Payload Context Clock and Reset
    .axis_data_clk (axis_data_clk),
    .axis_data_rst (axis_data_rst),
    // Payload Stream to User Logic: in
    .m_in_payload_tdata  (m_in_payload_tdata),
    .m_in_payload_tkeep  (m_in_payload_tkeep),
    .m_in_payload_tlast  (m_in_payload_tlast),
    .m_in_payload_tvalid (m_in_payload_tvalid),
    .m_in_payload_tready (m_in_payload_tready),
    // Context Stream to User Logic: in
    .m_in_context_tdata  (m_in_context_tdata),
    .m_in_context_tuser  (m_in_context_tuser),
    .m_in_context_tlast  (m_in_context_tlast),
    .m_in_context_tvalid (m_in_context_tvalid),
    .m_in_context_tready (m_in_context_tready),
    // Payload Stream from User Logic: out
    .s_out_payload_tdata  (s_out_payload_tdata),
    .s_out_payload_tkeep  (s_out_payload_tkeep),
    .s_out_payload_tlast  (s_out_payload_tlast),
    .s_out_payload_tvalid (s_out_payload_tvalid),
    .s_out_payload_tready (s_out_payload_tready),
    // Context Stream from User Logic: out
    .s_out_context_tdata  (s_out_context_tdata),
    .s_out_context_tuser  (s_out_context_tuser),
    .s_out_context_tlast  (s_out_context_tlast),
    .s_out_context_tvalid (s_out_context_tvalid),
    .s_out_context_tready (s_out_context_tready)
  );

  //---------------------------------------------------------------------------
  // User Logic
  //---------------------------------------------------------------------------

  `define RFNOC_CHDR_UTILS_PATH `"`UHD_FPGA_DIR/usrp3/lib/rfnoc/core/rfnoc_chdr_utils.vh`"
  `include `RFNOC_CHDR_UTILS_PATH

  // Logic --------------------------------------------------------------------

  // Number of 32-bit words in the DIFI header prepended by the DIFI framer
  localparam DIFI_HEADER_WORDS = 3'd7;
  localparam DIFI_HEADER_BYTES = 16'd28;

  //---------------------------------------------------------------------------
  // Per-packet EOB flag queue (context -> payload)
  //
  // The context path learns EOB from the CHDR header as soon as it arrives;
  // the payload path needs it at the *end* of the corresponding payload to
  // decide whether to flush or retain the residue. A small ring buffer
  // carries one flag per packet. The CHDR builder downstream consumes the
  // context header of packet k before packet k's payload completes, so the
  // flag is always written before the payload path blocks on it.
  //---------------------------------------------------------------------------

  reg  [63:0] eob_queue;
  reg  [5:0]  eobq_wr = 6'd0;
  reg  [5:0]  eobq_rd = 6'd0;
  wire        eobq_empty = (eobq_wr == eobq_rd);
  wire        eobq_full  = (eobq_wr + 6'd1 == eobq_rd);
  wire        eob_cur    = eob_queue[eobq_rd];

  //---------------------------------------------------------------------------
  // Per-packet DIFI type flag queue (payload -> context)
  //
  // The DIFI packet type lives in the first payload word, but the context
  // path must drop the CHDR header of non-data DIFI packets (e.g., the
  // standard flow signal context packets that compliant DIFI senders emit
  // periodically). The payload path classifies each packet at its first
  // word and queues the verdict; the context path stalls at each CHDR
  // header until the verdict for that packet is available. The input-side
  // FIFOs in the NoC shell guarantee the first payload word is obtainable
  // while the context header waits, so this cannot deadlock.
  //---------------------------------------------------------------------------

  reg  [63:0] type_ok_queue;
  reg  [5:0]  typeq_wr = 6'd0;
  reg  [5:0]  typeq_rd = 6'd0;
  wire        typeq_empty = (typeq_wr == typeq_rd);
  wire        typeq_full  = (typeq_wr + 6'd1 == typeq_rd);
  wire        type_ok_cur = type_ok_queue[typeq_rd];

  //---------------------------------------------------------------------------
  // Context path
  //
  // On each packet's CHDR header word: compute the incoming sample count N
  // from the length field, combine with the carried residue, and emit a
  // header whose length reflects the aligned output sample count. Packets
  // that would emit zero samples are dropped whole (all context words
  // swallowed). Non-header context words pass through unmodified.
  //---------------------------------------------------------------------------

  reg        ctxt_first_word = 1'b1;
  reg        ctxt_drop       = 1'b0;
  reg        ctxt_residue    = 1'b0;

  // Header field extraction (combinational, valid when ctxt_first_word)
  wire [15:0] hdr_len_in    = chdr_get_length(m_in_context_tdata);
  wire        hdr_eob       = chdr_get_eob(m_in_context_tdata);
  wire [15:0] hdr_overhead  = 16'd8 + (chdr_get_has_time(m_in_context_tdata) ? 16'd8 : 16'd0)
                              + ({11'd0, chdr_get_num_mdata(m_in_context_tdata)} << 3);
  // Incoming sample count N (32-bit words after the DIFI header)
  wire [15:0] hdr_n_samps   = (hdr_len_in - hdr_overhead - DIFI_HEADER_BYTES) >> 2;
  wire [16:0] hdr_avail     = {1'b0, hdr_n_samps} + {16'd0, ctxt_residue};
  // Guard N == 0 (host contract violation): drop, keep residue
  wire        hdr_res_next  = (hdr_n_samps == 16'd0) ? ctxt_residue :
                              (hdr_eob ? 1'b0 : hdr_avail[0]);
  wire [16:0] hdr_out_samps = hdr_avail - {16'd0, hdr_res_next};
  wire        hdr_zero_drop = (hdr_out_samps == 17'd0);
  // Non-data DIFI packet (verdict from the payload path's type queue)
  wire        hdr_type_drop = !type_ok_cur;
  wire        hdr_any_drop  = hdr_zero_drop || hdr_type_drop;
  wire [15:0] hdr_len_out   = hdr_overhead + (hdr_out_samps[15:0] << 2);

  // Header beats require space in the EOB queue and a type verdict
  wire ctxt_hdr_stall = ctxt_first_word && (eobq_full || typeq_empty);

  wire ctxt_in_beat = m_in_context_tvalid && m_in_context_tready;

  always @(posedge axis_data_clk) begin
    if (axis_data_rst) begin
      ctxt_first_word <= 1'b1;
      ctxt_drop       <= 1'b0;
      ctxt_residue    <= 1'b0;
      eobq_wr         <= 6'd0;
      typeq_rd        <= 6'd0;
    end else if (ctxt_in_beat) begin
      if (ctxt_first_word) begin
        ctxt_drop            <= hdr_any_drop && !m_in_context_tlast ? 1'b1 : 1'b0;
        // Residue is frozen across dropped non-data packets
        if (!hdr_type_drop)
          ctxt_residue       <= hdr_res_next;
        eob_queue[eobq_wr]   <= hdr_eob;
        eobq_wr              <= eobq_wr + 6'd1;
        typeq_rd             <= typeq_rd + 6'd1;
      end
      if (m_in_context_tlast) begin
        ctxt_first_word <= 1'b1;
        ctxt_drop       <= 1'b0;
      end else begin
        ctxt_first_word <= 1'b0;
      end
    end
  end

  wire ctxt_emitting = ctxt_first_word ? !hdr_any_drop : !ctxt_drop;

  assign s_out_context_tdata  = ctxt_first_word ?
    chdr_set_length(m_in_context_tdata, hdr_len_out) : m_in_context_tdata;
  assign s_out_context_tuser  = m_in_context_tuser;
  assign s_out_context_tlast  = m_in_context_tlast;
  assign s_out_context_tvalid = m_in_context_tvalid && ctxt_emitting && !ctxt_hdr_stall;
  assign m_in_context_tready  = !ctxt_hdr_stall &&
                                (ctxt_emitting ? s_out_context_tready : 1'b1);

  //---------------------------------------------------------------------------
  // Payload path
  //
  // Per packet: skip the 7 DIFI header words, then stream samples through a
  // one-deep skid buffer so that tlast placement (and residue retention)
  // can be decided when the final input sample arrives. The residue sample
  // from an odd packet is prepended to the next packet's samples.
  //---------------------------------------------------------------------------

  localparam PS_SKIP   = 3'd0;
  localparam PS_STREAM = 3'd1;
  localparam PS_FLUSH  = 3'd2;
  localparam PS_DROP   = 3'd3;  // consuming a non-data DIFI packet
  localparam PS_DRAIN  = 3'd4;  // packet ended early: pop the EOB flag

  reg [2:0]  pyld_state = PS_SKIP;
  reg [2:0]  skip_count = 3'd0;
  reg [31:0] skid_data;
  reg        skid_valid = 1'b0;
  reg [31:0] residue_data;
  reg        residue_valid = 1'b0;
  reg        avail_parity  = 1'b0;  // parity of residue + samples received

  // DIFI packet type check on the first payload word. The DIFI framer's
  // byte packing places the VITA packet-type nibble at bits [23:20] of the
  // word as seen on this bus. Only signal data packets (type 0x1) carry
  // samples; anything else (standard/version context packets, types
  // 0x4/0x5) is dropped whole.
  wire pyld_word0     = (pyld_state == PS_SKIP) && (skip_count == 3'd0);
  wire difi_type_ok   = (m_in_payload_tdata[23:20] == 4'h1);

  // At the final input sample: parity including this sample
  wire pyld_par_after = avail_parity ^ 1'b1;
  // Retain the final sample as residue? (needs EOB flag; see stall below)
  wire pyld_retain    = pyld_par_after && !eob_cur;
  // Final-beat decisions require the EOB flag for this packet
  wire pyld_last_stall = m_in_payload_tlast && eobq_empty;

  wire pyld_in_beat  = m_in_payload_tvalid && m_in_payload_tready;
  wire pyld_out_beat = s_out_payload_tvalid && s_out_payload_tready;

  always @(posedge axis_data_clk) begin
    if (axis_data_rst) begin
      pyld_state    <= PS_SKIP;
      skip_count    <= 3'd0;
      skid_valid    <= 1'b0;
      residue_valid <= 1'b0;
      avail_parity  <= 1'b0;
      eobq_rd       <= 6'd0;
      typeq_wr      <= 6'd0;
    end else begin
      case (pyld_state)
        PS_SKIP: begin
          if (pyld_in_beat) begin
            if (pyld_word0) begin
              // Classify the packet and queue the verdict for the context
              // path (which stalls on it before emitting the CHDR header).
              type_ok_queue[typeq_wr] <= difi_type_ok;
              typeq_wr                <= typeq_wr + 6'd1;
            end
            if (pyld_word0 && !difi_type_ok) begin
              // Non-data DIFI packet: consume it whole, emit nothing
              skip_count <= 3'd0;
              pyld_state <= m_in_payload_tlast ? PS_DRAIN : PS_DROP;
            end else if (m_in_payload_tlast) begin
              // Packet ended inside the DIFI header (N == 0 contract
              // violation): discard, keep residue.
              skip_count <= 3'd0;
              pyld_state <= PS_DRAIN;
            end else if (skip_count == DIFI_HEADER_WORDS - 3'd1) begin
              // Last DIFI header word: enter streaming, preload residue
              skip_count   <= 3'd0;
              pyld_state   <= PS_STREAM;
              skid_data    <= residue_data;
              skid_valid   <= residue_valid;
              avail_parity <= residue_valid;
              residue_valid <= 1'b0;
            end else begin
              skip_count <= skip_count + 3'd1;
            end
          end
        end

        PS_STREAM: begin
          if (pyld_in_beat) begin
            avail_parity <= avail_parity ^ 1'b1;
            if (m_in_payload_tlast) begin
              eobq_rd <= eobq_rd + 6'd1;
              if (pyld_retain) begin
                // Final sample becomes the residue; packet output (if any)
                // ended with the skid emitted this beat.
                residue_data  <= m_in_payload_tdata;
                residue_valid <= 1'b1;
                skid_valid    <= 1'b0;
                pyld_state    <= PS_SKIP;
              end else begin
                // Final sample must be emitted with tlast: park it in the
                // skid and flush next.
                skid_data  <= m_in_payload_tdata;
                skid_valid <= 1'b1;
                pyld_state <= PS_FLUSH;
              end
            end else begin
              skid_data  <= m_in_payload_tdata;
              skid_valid <= 1'b1;
            end
          end
        end

        PS_FLUSH: begin
          if (pyld_out_beat) begin
            skid_valid <= 1'b0;
            pyld_state <= PS_SKIP;
          end
        end

        PS_DROP: begin
          if (pyld_in_beat && m_in_payload_tlast)
            pyld_state <= PS_DRAIN;
        end

        PS_DRAIN: begin
          // The context path may not have written this packet's EOB flag
          // yet (it waits on our type verdict); pop it as soon as it lands.
          if (!eobq_empty) begin
            eobq_rd    <= eobq_rd + 6'd1;
            pyld_state <= PS_SKIP;
          end
        end

        default: pyld_state <= PS_SKIP;
      endcase
    end
  end

  // Output/input handshake wiring
  //  - PS_SKIP:   consume freely (word 0 needs type-queue space), emit nothing
  //  - PS_STREAM: emitting the skid requires an input beat displacing it;
  //               the retained-final-sample case emits the skid with tlast
  //  - PS_FLUSH:  emit the parked final sample with tlast
  //  - PS_DROP:   consume freely, emit nothing
  //  - PS_DRAIN:  no input, no output; waiting to pop the EOB flag
  assign s_out_payload_tdata  = (pyld_state == PS_FLUSH) ? skid_data :
                                skid_valid ? skid_data : 32'b0;
  assign s_out_payload_tkeep  = 1'b1;
  assign s_out_payload_tlast  = (pyld_state == PS_FLUSH) ? 1'b1 :
                                (m_in_payload_tlast && pyld_retain);
  assign s_out_payload_tvalid =
    (pyld_state == PS_FLUSH)  ? 1'b1 :
    (pyld_state == PS_STREAM) ? (skid_valid && m_in_payload_tvalid && !pyld_last_stall) :
    1'b0;
  assign m_in_payload_tready  =
    (pyld_state == PS_SKIP)   ? (pyld_word0 ? !typeq_full : 1'b1) :
    (pyld_state == PS_STREAM) ? (!pyld_last_stall && (skid_valid ? s_out_payload_tready : 1'b1)) :
    (pyld_state == PS_DROP)   ? 1'b1 :
    1'b0; // PS_FLUSH / PS_DRAIN: hold input

  // Fixed-function block: no user registers
  assign m_ctrlport_resp_ack  = 1'b0;
  assign m_ctrlport_resp_data = 32'b0;

endmodule // rfnoc_block_difi_deframer


`default_nettype wire
