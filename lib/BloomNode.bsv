/* This module is a double-buffer BRAM writer for BloomFilter */

package BloomNode;

import FIFO::*;
import Vector::*;
import BRAM::*;
import BRAMFIFO::*;
import BramCtl::*;
import FIFOLI::*;
import Serializer::*;

interface BloomNodeIfc;
    method Action enq(Bit#(32) d);
    method Action dram_enq(Bit#(512) d);
    method ActionValue#(Bit#(16)) dram_write_req;
    method ActionValue#(Bit#(16)) dram_read_req;
    method ActionValue#(Bit#(512)) dram_write_data_get;
endinterface

(* conflict_free = "read_from_bram, write_back_to_bram_req, merge_dram_write_data, read_data_from_DRAM" *)
module mkBloomNode (BloomNodeIfc);

    FIFO#(Bit#(32)) inQ <- mkFIFO;
    FIFO#(Bit#(16)) dram_write_reqQ <- mkFIFO;
    FIFO#(Bit#(16)) dram_read_reqQ <- mkFIFO;

    Reg#(Bit#(1)) pre_handle <- mkReg(0);
    Reg#(Bit#(1)) app_handle <- mkReg(0);
    Reg#(Bit#(1)) dram_write_handle <- mkReg(1);

    FIFO#(Bit#(32)) toApplierQ <- mkSizedBRAMFIFO(2048);
    FIFOLI#(Bit#(32), 2) pre_appQ <- mkFIFOLI;
    FIFOLI#(Bit#(32), 2) post_appQ <- mkFIFOLI;

    FIFOLI#(Bit#(128), 4) dram_readQ <- mkFIFOLI;
    FIFOLI#(Bit#(128), 3) dram_writeQ <- mkFIFOLI;

    /* FIFOLI#(Bit#(32), 4) toApplierQ <- mkFIFOLI;
     * FIFOLI#(Bit#(128), 4) dram_readQ <- mkFIFOLI;
     * FIFOLI#(Bit#(128), 4) dram_writeQ <- mkFIFOLI; */

    Reg#(Bit#(16)) prepat_cache <- mkReg(0);
    Reg#(Bit#(16)) app_cache <- mkReg(0);

    SerializerIfc#(512, 4) serial_dramQ <- mkSerializer;

    Vector#(2, FIFO#(Bit#(32))) toBramQ <- replicateM(mkFIFO);

    Reg#(Bit#(2)) pre_first_flag <- mkReg(0);
    Reg#(Bit#(2)) app_first_flag <- mkReg(0);

    FIFOLI#(Bit#(16), 2) next_read_dataQ <- mkFIFOLI;
    FIFOLI#(Bit#(16), 2) applier_doneQ <- mkFIFOLI;

    Vector#(2, BramCtlIfc#(128, 512, 9)) bram_ctl <- replicateM(mkBramCtl);
    Vector#(2, Reg#(Bit#(2))) bram_ready <- replicateM(mkReg(1));

    rule prepatcher;
        inQ.deq;
        let d = inQ.first;
        Bit#(16) upper = truncateLSB(d);
        
        // if first 16 bits are diff -> read from dram
        if (prepat_cache != upper) begin
            prepat_cache <= upper;

            if (pre_first_flag == 2) begin // key point for this algorithm
                next_read_dataQ.enq(upper);
                /* dram_read_reqQ.enq(upper); */
            end else begin
                pre_first_flag <= pre_first_flag + 1;
            end
        end

        pre_appQ.enq(d);
    endrule

    rule bridge_app;
        pre_appQ.deq;
        toApplierQ.enq(pre_appQ.first);
    endrule

    rule bridge_app_post;
        toApplierQ.deq;
        post_appQ.enq(toApplierQ.first);
    endrule

    rule applier;
        post_appQ.deq;
        let d = post_appQ.first;
        Bit#(16) upper = truncateLSB(d);
        Bit#(1) handle = app_handle;

        if (app_cache != upper) begin
            app_handle <= app_handle + 1;
            app_cache <= upper;
            handle = handle + 1;

            if (app_first_flag == 1) begin
                applier_doneQ.enq(app_cache);
            end else begin
                app_first_flag <= app_first_flag + 1;
            end
        end

        toBramQ[handle].enq(d);
    endrule

    Vector#(2, FIFO#(Bit#(16))) bram_dataQ <- replicateM(mkSizedFIFO(4));
    for (Bit#(8) i = 0; i < 2; i = i + 1) begin
        rule read_from_bram(bram_ready[i] == 1); // read & write
            toBramQ[i].deq;
            Bit#(32) d = toBramQ[i].first;
            Bit#(16) data = truncate(d);
            Bit#(9) bram_idx = truncateLSB(data);

            bram_ctl[i].read_req(bram_idx);
            bram_dataQ[i].enq(data);
        endrule

        rule write_back_to_bram(bram_ready[i] == 1);
            bram_dataQ[i].deq;
            Bit#(128) d <- bram_ctl[i].get;

            Bit#(16) data = bram_dataQ[i].first;
            Bit#(7)  bram_data = truncate(data);
            Bit#(9)  bram_idx = truncateLSB(data);

            d[bram_data] = 1;
            bram_ctl[i].write_req(bram_idx, d);
        endrule
    end


    Reg#(Bit#(1)) bram_control_handle <- mkReg(1);
    Reg#(Bit#(10)) bram_read_req_cnt <- mkReg(0);
    Reg#(Bit#(10)) bram_read_done_cnt <- mkReg(0);

    Reg#(Bit#(10)) bram_write_req_cnt <- mkReg(0);

    DeSerializerIfc#(128, 4) deserial_dram <- mkDeSerializer;
    FIFO#(Bit#(512)) dram_deserialQ <- mkFIFO;

    rule check_bram_write_done(bram_ready[bram_control_handle] == 1);
        next_read_dataQ.deq;
        applier_doneQ.deq;

        dram_write_reqQ.enq(applier_doneQ.first);
        dram_read_reqQ.enq(next_read_dataQ.first);
        bram_ready[bram_control_handle] <= 0;
    endrule

    // Write to DRAM
    rule write_back_to_bram_req(bram_ready[bram_control_handle] == 0 && bram_read_req_cnt < 512);
        Bit#(1) handle = bram_control_handle;
        bram_ctl[handle].read_req(truncate(bram_read_req_cnt));
        bram_read_req_cnt <= bram_read_req_cnt + 1;
    endrule
    rule merge_dram_write_data(bram_ready[bram_control_handle] == 0);
        Bit#(128) d <- bram_ctl[bram_control_handle].get;
        if (bram_read_done_cnt == 511 && bram_read_req_cnt == 512) begin
            bram_read_req_cnt <= 0;
            bram_read_done_cnt <= 0;
            bram_ready[bram_control_handle] <= 2;
        end else begin
            bram_read_done_cnt <= bram_read_done_cnt + 1;
        end
        dram_writeQ.enq(d);
    endrule
    rule out_dram;
        dram_writeQ.deq;
        deserial_dram.put(dram_writeQ.first);
    endrule
    rule deserial_to_out;
        Bit#(512) d <- deserial_dram.get;
        dram_deserialQ.enq(d);
    endrule

    // Read from DRAM
    rule read_data_from_DRAM(bram_ready[bram_control_handle] == 2 && bram_write_req_cnt < 512);
        dram_readQ.deq;
        Bit#(128) d = dram_readQ.first;
        Bit#(1) handle = bram_control_handle;
        bram_ctl[handle].write_req(truncate(bram_write_req_cnt), d);

        if (bram_write_req_cnt == 511) begin // write to BRAM is done
            bram_write_req_cnt <= 0;
            bram_control_handle <= bram_control_handle + 1;
            bram_ready[bram_control_handle] <= 1;
        end else begin
            bram_write_req_cnt <= bram_write_req_cnt + 1;
        end
    endrule

    rule in_dram_read;
        let d <- serial_dramQ.get;
        dram_readQ.enq(d);
    endrule

    method Action enq(Bit#(32) d);
        inQ.enq(d);
    endmethod
    method Action dram_enq(Bit#(512) d);
        serial_dramQ.put(d);
    endmethod
    method ActionValue#(Bit#(16)) dram_write_req;
        dram_write_reqQ.deq;
        return dram_write_reqQ.first;
    endmethod
    method ActionValue#(Bit#(512)) dram_write_data_get;
        dram_deserialQ.deq;
        return dram_deserialQ.first;
    endmethod
    method ActionValue#(Bit#(16)) dram_read_req;
        dram_read_reqQ.deq;
        return dram_read_reqQ.first;
    endmethod
endmodule
endpackage
