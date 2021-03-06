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
    method Action enq(Bit#(64) d);
    method Action dram_enq(Bit#(512) d);
    method ActionValue#(Bit#(16)) dram_write_req;
    method ActionValue#(Bit#(16)) dram_read_req;
    method ActionValue#(Bit#(512)) dram_write_data_get;
endinterface

(* conflict_free = "read_from_bram, write_back_to_bram_req, merge_dram_write_data, read_data_from_DRAM" *)
module mkBloomNode (BloomNodeIfc);

    FIFO#(Bit#(64)) inQ <- mkFIFO;
    FIFO#(Bit#(16)) dram_write_reqQ <- mkFIFO;
    FIFO#(Bit#(16)) dram_read_reqQ <- mkFIFO;

    Reg#(Bit#(1)) pre_handle <- mkReg(0);
    Reg#(Bit#(1)) app_handle <- mkReg(0);
    Reg#(Bit#(1)) dram_write_handle <- mkReg(1);

    FIFO#(Bit#(64)) toApplierQ <- mkSizedBRAMFIFO(2048);
    FIFOLI#(Bit#(64), 2) pre_appQ <- mkFIFOLI;
    FIFOLI#(Bit#(64), 2) post_appQ <- mkFIFOLI;

    FIFOLI#(Bit#(512), 4) dram_readQ <- mkFIFOLI;
    FIFOLI#(Bit#(512), 3) dram_writeQ <- mkFIFOLI;

    Reg#(Bit#(16)) prepat_cache <- mkReg(0);
    Reg#(Bit#(16)) app_cache <- mkReg(0);

    Vector#(2, FIFO#(Bit#(64))) toBramQ <- replicateM(mkFIFO);

    Reg#(Bit#(2)) pre_first_flag <- mkReg(0);
    Reg#(Bit#(2)) app_first_flag <- mkReg(0);

    FIFOLI#(Bit#(16), 2) next_read_dataQ <- mkFIFOLI;
    FIFOLI#(Bit#(16), 2) applier_doneQ <- mkFIFOLI;

    Vector#(4, BramCtlIfc#(512, 128, 7)) bram_ctl <- replicateM(mkBramCtl);
    Vector#(2, Reg#(Bit#(2))) bram_ready <- replicateM(mkReg(1));

    rule prepatcher;
        inQ.deq;
        let d = inQ.first;
        Bit#(16) d_lsb = truncate(d);

        // if first 16 bits are diff -> read from dram
        if (prepat_cache != d_lsb) begin
            prepat_cache <= d_lsb;
            if (pre_first_flag == 2) begin // key point for this algorithm
                next_read_dataQ.enq(d_lsb);
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
        Bit#(16) d_lsb = truncate(d);
        Bit#(1) handle = app_handle;

        if (app_cache != d_lsb) begin
            app_handle <= app_handle + 1;
            app_cache <= d_lsb;
            handle = handle + 1;

            if (app_first_flag == 1) begin
                applier_doneQ.enq(app_cache);
            end else begin
                app_first_flag <= app_first_flag + 1;
            end
        end

        toBramQ[handle].enq(d);
    endrule

    Vector#(4, FIFO#(Bit#(16))) bram_dataQ <- replicateM(mkSizedFIFO(4));
    for (Bit#(8) i = 0; i < 2; i = i + 1) begin
        rule read_from_bram(bram_ready[i] == 1); // read & write
            toBramQ[i].deq;
            Bit#(64) d = toBramQ[i].first;
            Bit#(32) d_1 = truncate(d);
            Bit#(32) d_2 = truncateLSB(d);

            Bit#(16) data_1 = truncateLSB(d_1);
            Bit#(7) bram_idx_1 = truncate(data_1);

            Bit#(16) data_2 = truncateLSB(d_2);
            Bit#(7) bram_idx_2 = truncate(data_2);

            bram_ctl[i * 2].read_req(bram_idx_1);
            bram_ctl[i * 2 + 1].read_req(bram_idx_2);
            bram_dataQ[i * 2].enq(data_1);
            bram_dataQ[i * 2 + 1].enq(data_2);
        endrule
    end

    for (Bit#(8) i = 0; i < 4; i = i + 1) begin
        rule write_back_to_bram(bram_ready[i / 2] == 1);
            bram_dataQ[i].deq;
            Bit#(512) d <- bram_ctl[i].get;

            Bit#(16) data = bram_dataQ[i].first;
            Bit#(9)  bram_data = truncateLSB(data);
            Bit#(7)  bram_idx = truncate(data);

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
    rule write_back_to_bram_req(bram_ready[bram_control_handle] == 0 && bram_read_req_cnt < 128);
        Bit#(1) handle = bram_control_handle;
        if (handle == 0) begin
            bram_ctl[0].read_req(truncate(bram_read_req_cnt));
            bram_ctl[1].read_req(truncate(bram_read_req_cnt));
        end else begin
            bram_ctl[2].read_req(truncate(bram_read_req_cnt));
            bram_ctl[3].read_req(truncate(bram_read_req_cnt));
        end
        bram_read_req_cnt <= bram_read_req_cnt + 1;
    endrule
    rule merge_dram_write_data(bram_ready[bram_control_handle] == 0);
        Bit#(1) handle = bram_control_handle;
        Bit#(512) d1 = 0;
        Bit#(512) d2 = 0;
        if (handle == 0) begin
            d1 <- bram_ctl[0].get;
            d2 <- bram_ctl[1].get;
        end else begin
            d1 <- bram_ctl[2].get;
            d2 <- bram_ctl[3].get;
        end

        if (bram_read_done_cnt == 127 && bram_read_req_cnt == 128) begin
            bram_read_req_cnt <= 0;
            bram_read_done_cnt <= 0;
            bram_ready[bram_control_handle] <= 2;
        end else begin
            bram_read_done_cnt <= bram_read_done_cnt + 1;
        end
        dram_writeQ.enq(d1 | d2);
    endrule

    // Read from DRAM
    rule read_data_from_DRAM(bram_ready[bram_control_handle] == 2 && bram_write_req_cnt < 128);
        dram_readQ.deq;
        Bit#(512) d = dram_readQ.first;
        Bit#(1) handle = bram_control_handle;
        if (handle == 0) begin
            bram_ctl[0].write_req(truncate(bram_write_req_cnt), d);
            bram_ctl[1].write_req(truncate(bram_write_req_cnt), d);
        end else begin
            bram_ctl[2].write_req(truncate(bram_write_req_cnt), d);
            bram_ctl[3].write_req(truncate(bram_write_req_cnt), d);
        end

        if (bram_write_req_cnt == 127) begin // write to BRAM is done
            bram_write_req_cnt <= 0;
            bram_control_handle <= bram_control_handle + 1;
            bram_ready[bram_control_handle] <= 1;
        end else begin
            bram_write_req_cnt <= bram_write_req_cnt + 1;
        end
    endrule

    method Action enq(Bit#(64) d);
        inQ.enq(d);
    endmethod
    method Action dram_enq(Bit#(512) d);
        dram_readQ.enq(d);
    endmethod
    method ActionValue#(Bit#(16)) dram_write_req;
        dram_write_reqQ.deq;
        return dram_write_reqQ.first;
    endmethod
    method ActionValue#(Bit#(512)) dram_write_data_get;
        dram_writeQ.deq;
        return dram_writeQ.first;
    endmethod
    method ActionValue#(Bit#(16)) dram_read_req;
        dram_read_reqQ.deq;
        return dram_read_reqQ.first;
    endmethod
endmodule
endpackage
