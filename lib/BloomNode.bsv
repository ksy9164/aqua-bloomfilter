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

module mkBloomNode (BloomNodeIfc);

    FIFO#(Bit#(32)) inQ <- mkFIFO;
    FIFO#(Bit#(16)) dram_write_reqQ <- mkFIFO;
    FIFO#(Bit#(16)) dram_read_reqQ <- mkFIFO;

    Reg#(Bit#(1)) pre_handle <- mkReg(0);
    Reg#(Bit#(1)) app_handle <- mkReg(0);
    Reg#(Bit#(1)) dram_read_handle <- mkReg(1);
    Reg#(Bit#(1)) dram_write_handle <- mkReg(1);

    FIFO#(Bit#(32)) toApplierQ <- mkSizedBRAMFIFO(2048);

    Reg#(Bit#(16)) prepat_cache <- mkReg(0);
    Reg#(Bit#(16)) app_cache <- mkReg(0);

    SerializerIfc#(512, 4) serial_dramQ <- mkSerializer;
    FIFO#(Bit#(128)) dram_readQ <- mkSizedBRAMFIFO(512);
    FIFO#(Bit#(128)) dram_writeQ <- mkSizedBRAMFIFO(512);
    Vector#(2, FIFO#(Bit#(32))) toBramQ <- replicateM(mkFIFO);

    Reg#(Bit#(2)) pre_first_flag <- mkReg(0);
    Reg#(Bit#(2)) app_first_flag <- mkReg(0);

    Vector#(2, BramCtlIfc#(128, 512, 9)) bram_ctl <- replicateM(mkBramCtl);
    Vector#(2, Reg#(Bit#(2))) bram_ready <- replicateM(mkReg(1));

    rule prepatcher;
        inQ.deq;
        let d = inQ.first;
        Bit#(16) upper = truncateLSB(d);
        
        // if first 16 bits are diff -> read from dram
        if (prepat_cache != upper) begin
            $display("pre diff get");
            prepat_cache <= upper;
            pre_handle <= pre_handle + 1;

            if (pre_first_flag == 2) begin // key point for this algorithm
                dram_read_reqQ.enq(upper);
            end else begin
                pre_first_flag <= pre_first_flag + 1;
            end
        end

        toApplierQ.enq(d);
    endrule

    rule applier(bram_ready[app_handle] == 1);
        toApplierQ.deq;
        let d = toApplierQ.first;
        Bit#(16) upper = truncateLSB(d);
        Bit#(1) handle = app_handle;

        if (app_cache != upper) begin
            $display("app diff get");
            app_handle <= app_handle + 1;
            app_cache <= upper;
            handle = handle + 1;

            if (app_first_flag == 1) begin
                bram_ready[app_handle] <= 0;
                dram_write_reqQ.enq(app_cache);
            end else begin
                app_first_flag <= app_first_flag + 1;
            end
        end

        toBramQ[handle].enq(d);
    endrule

    Reg#(Bit#(10)) bram_read_req_cnt <- mkReg(0);
    Reg#(Bit#(10)) bram_read_done_cnt <- mkReg(0);

    Reg#(Bit#(10)) bram_write_req_cnt <- mkReg(0);
    Reg#(Bit#(10)) bram_write_done_cnt <- mkReg(0);

    Vector#(2, FIFO#(Bit#(16))) bram_dataQ <- replicateM(mkFIFO);
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

    DeSerializerIfc#(128, 4) deserial_dram <- mkDeSerializer;

    rule write_to_dram(bram_ready[dram_write_handle] == 0 && bram_read_req_cnt < 512); // write back to DRAM
        Bit#(1) handle = dram_write_handle;
        bram_ctl[handle].read_req(truncate(bram_read_req_cnt));
        bram_read_req_cnt <= bram_read_req_cnt + 1;
    endrule
    rule merge_dram_write_data(bram_ready[dram_write_handle] == 0);
        Bit#(128) d <- bram_ctl[dram_write_handle].get;
        if (bram_read_done_cnt == 511 && bram_read_req_cnt == 512) begin
            dram_write_handle <= dram_write_handle + 1;
            bram_read_req_cnt <= 0;
            bram_read_done_cnt <= 0;
            bram_ready[dram_write_handle] <= 2;
        end else begin
            bram_read_done_cnt <= bram_read_done_cnt + 1;
        end
        dram_writeQ.enq(d);
    endrule
    
    rule out_dram;
        dram_writeQ.deq;
        deserial_dram.put(dram_writeQ.first);
    endrule

    rule put_dram_read_to_bram(bram_ready[dram_read_handle] == 2 && bram_write_req_cnt < 512);
        dram_readQ.deq;
        Bit#(128) d = dram_readQ.first;

        Bit#(1) handle = dram_read_handle;
        bram_ctl[handle].write_req(truncate(bram_write_req_cnt), d);
        $display("write to bram gogo ");

        if (bram_write_req_cnt == 511) begin // write to BRAM is done
            $display("bram_write_!@# done?!");
            bram_write_req_cnt <= 0;
            dram_read_handle <= dram_read_handle + 1;
            bram_ready[dram_read_handle] <= 1;
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
        Bit#(512) d <- deserial_dram.get;
        return d;
    endmethod
    method ActionValue#(Bit#(16)) dram_read_req;
        dram_read_reqQ.deq;
        return dram_read_reqQ.first;
    endmethod
endmodule
endpackage
