import FIFO::*;
import FIFOF::*;
import Clocks::*;
import Vector::*;
import BRAM::*;
import BRAMFIFO::*;
import BramCtl::*;

import PcieCtrl::*;
import DRAMController::*;
import Serializer::*;
import FIFOLI::*;
import MultiN::*;
import BloomHash::*;
import BloomNode::*;

import BLRadix::*;
import DRAMarbiter::*;

interface HwMainIfc;
endinterface

module mkHwMain#(PcieUserIfc pcie, DRAMUserIfc dram)
    (HwMainIfc);
    Reg#(Bit#(32)) file_size <- mkReg(2550000);
    FIFOLI#(Tuple2#(Bit#(20), Bit#(32)), 2) pcie_reqQ <- mkFIFOLI;

    SerializerIfc#(128, 2) serial_dmaQ <- mkSerializer;
    SerializerIfc#(256, 2) serial_to_radixQ <- mkSerializer;

    FIFOLI#(Bit#(32), 2) dmaReadReqQ <- mkFIFOLI;
    FIFOLI#(Bit#(32), 2) dmaWriteReqQ <- mkFIFOLI;
    FIFOLI#(Bit#(1), 2) dmaWriteDoneSignalQ <- mkFIFOLI;

    Reg#(Bit#(32)) readCnt <- mkReg(0);

    FIFOLI#(Tuple2#(PcieCtrl::IOReadReq, Bit#(1)), 2) pcie_write_doneQ <- mkFIFOLI;
    Vector#(8, BloomHashIfc) hashfuncQ <- replicateM(mkJsHash);
    MultiOneToEightIfc#(Bit#(64)) toHashMultiQ <- mkMultiOnetoEight;

    hashfuncQ[0] <- mkJsHash;
    hashfuncQ[1] <- mkAddictiveHash;
    hashfuncQ[2] <- mkSdbmHash;
    hashfuncQ[3] <- mkDjbHash;
    hashfuncQ[4] <- mkBernsteinHash;
    hashfuncQ[5] <- mkBkdrHash;
    hashfuncQ[6] <- mkDekHash;
    hashfuncQ[7] <- mkRotatingHash;

    DRAMarbiterlIfc#(2048, 5) dramArbiter <- mkDRAMarbiter(dram);

    Vector#(2, FIFOLI#(Bit#(64), 3)) spread_inputQ <- replicateM(mkFIFOLI);

    Vector#(4, FIFO#(Bit#(64))) merge_st1Q <- replicateM(mkFIFO);
    Vector#(2, FIFO#(Bit#(128))) merge_st2Q <- replicateM(mkFIFO);

    Vector#(2, BloomNodeIfc) node <- replicateM(mkBloomNode);
    Reg#(Bit#(64)) clock_cnt <- mkReg(0); 
    Reg#(Bit#(64)) run_cnt <- mkReg(0); 

    rule getDataFromHost;
        let w <- pcie.dataReceive;
        let a = w.addr;
        let d = w.data;
        pcie_reqQ.enq(tuple2(a, d));
        if (clock_cnt != 0) begin
            $display("clock %d , run %d ratio %d", clock_cnt, run_cnt, ((run_cnt * 4000) / clock_cnt));
        end
    endrule

    rule getPCIeData; // get from HOST
        pcie_reqQ.deq;
        Bit#(20) a = tpl_1(pcie_reqQ.first);
        Bit#(32) d = tpl_2(pcie_reqQ.first);

        let off = (a>>2);
        if (off == 1) begin // Log Data In
            dmaReadReqQ.enq(d);
        end else begin
            $display("Wrong PCIe Signal");
        end
    endrule

    rule getReadReq(readCnt == 0);
        dmaReadReqQ.deq;
        Bit#(32) cnt = dmaReadReqQ.first;
        pcie.dmaReadReq(0, truncate(cnt)); // offset, words
        readCnt <= cnt;
    endrule

    rule getDataFromDMA(readCnt != 0);
        Bit#(128) rd <- pcie.dmaReadWord;
        if (readCnt - 1 == 0) begin
            dmaWriteDoneSignalQ.enq(1);
        end
        readCnt <= readCnt - 1;
        serial_dmaQ.put(rd);
    endrule

    rule scatterKmer;
        Bit#(64) d <- serial_dmaQ.get;
        toHashMultiQ.enq(d);
    endrule

    Reg#(Bit#(32)) temp_val <- mkReg(0);

    for (Bit#(8) i = 0; i < 8; i = i + 1) begin
        rule hashing;
            Bit#(64) kmer <- toHashMultiQ.get[i].get;
            hashfuncQ[i].enq(kmer);
        endrule
    end

    /* Merging */
    for (Bit#(8) i = 0; i < 4; i = i + 1) begin
        rule hashGet_merge_step_1;
            Bit#(32) d1 <- hashfuncQ[i * 2].get;
            Bit#(32) d2 <- hashfuncQ[i * 2 + 1].get;

            Bit#(64) val = zeroExtend(d1);
            val = val << 32;
            val = val | zeroExtend(d2);
            merge_st1Q[i].enq(val);
        endrule
    end
    for (Bit#(8) i = 0; i < 2; i = i + 1) begin
        rule hashGet_merge_step_2;
            merge_st1Q[i * 2].deq;
            merge_st1Q[i * 2 + 1].deq;
            Bit#(128) d1 = zeroExtend(merge_st1Q[i * 2].first);
            Bit#(128) d2 = zeroExtend(merge_st1Q[i * 2 + 1].first);
            d2 = d2 << 64;

            merge_st2Q[i].enq(d1 | d2);
        endrule
    end

    rule hashGet_merge_step_3;
        merge_st2Q[0].deq;
        merge_st2Q[1].deq;
        Bit#(256) d1 = zeroExtend(merge_st2Q[0].first);
        Bit#(256) d2 = zeroExtend(merge_st2Q[1].first);
        d2 = d2 << 128;
        serial_to_radixQ.put(d1 | d2);
    endrule
//////////////// Sorting Part ///////////////////

    Reg#(Bit#(7)) dram_write_cnt <- mkReg(0);
    BLRadixIfc#(8,3,4,Bit#(32),0,8) radixFirst <- mkBLRadix;
    BLRadixIfc#(10,3,4,Bit#(32),8,7) radixSecond <- mkBLRadix;

    DeSerializerIfc#(128, 4) first_sort_desQ <- mkDeSerializer;
    SerializerIfc#(512, 4) serial_secondsQ <- mkSerializer;

    FIFO#(Bit#(512)) temp_save_to_firstOutQ <- mkSizedBRAMFIFO(1024);
    FIFO#(Vector#(4, Bit#(32))) toUploader <- mkSizedBRAMFIFO(4096);
    FIFO#(Bit#(8)) lsbQ <- mkSizedFIFO(21);
    FIFO#(Bit#(8)) dram_req_idxQ <- mkSizedFIFO(11);
    FIFO#(Bit#(8)) save_addressQ <- mkSizedFIFO(11);

    BramCtlIfc#(10, 256, 8) bram_ctl <- mkBramCtl;

    // it makes stall
    FIFO#(Bit#(32)) dram_fullQ <- mkSizedBRAMFIFO(11); // fix needed
    FIFO#(Tuple4#(Bit#(32), Bit#(1), Bit#(16), Bit#(3))) radix_dram_reqQ <- mkSizedBRAMFIFO(512);

    Integer base_idx = 1024*1024*512;

    Reg#(Bit#(32)) start_cnt <- mkReg(0); 

    Reg#(Bit#(32)) burstTotal <- mkReg(0);
    Reg#(Bit#(32)) burstTotal_sec <- mkReg(0);
    Reg#(Bit#(32)) startCycle <- mkReg(0);
    rule flushBurstReady;
        let d <- radixFirst.burstReady;
        burstTotal <= burstTotal + zeroExtend(d);
    endrule

// start
    rule flushBurstReady_second;
        let d <- radixSecond.burstReady;
        burstTotal_sec <= burstTotal_sec + zeroExtend(d);
    endrule

    rule inputData;
        Bit#(128) t <- serial_to_radixQ.get;
        Vector#(4,Bit#(32)) ind;
        ind[0] = t[127:96];
        ind[1] = t[95:64];
        ind[2] = t[63:32];
        ind[3] = t[31:0];
        run_cnt <= run_cnt + 1;
        start_cnt <= 1;
        radixFirst.enq(ind);
    endrule

    rule get_data;
        radixFirst.deq;
        Vector#(4,Bit#(32)) ind = radixFirst.first;
        Bit#(128) t = 0;
        t[127:96] = ind[0];
        t[95:64] = ind[1];
        t[63:32] = ind[2];
        t[31:0] = ind[3];
        if (dram_write_cnt == 127) begin
            /* $display("Change!! "); */
            lsbQ.enq(truncate(t)); // to write req to DRAM (2KB each)
        end  
        dram_write_cnt <= dram_write_cnt + 1;
        first_sort_desQ.put(t);
        /* temp_save_to_firstOutQ.enq(t); // 8KB buffer for 2KB */
    endrule

    rule get512andsave;
        let d <- first_sort_desQ.get;
        temp_save_to_firstOutQ.enq(d);
    endrule

    rule req_DRAM_address; // when 2KB comes
        lsbQ.deq;
        let d = lsbQ.first;

        bram_ctl.read_req(d); // to get DRAM current idx
        save_addressQ.enq(d);
    endrule

    rule calculate_DRAM_address;
        Bit#(10) t <- bram_ctl.get;
        Bit#(32) d = zeroExtend(t);
        save_addressQ.deq;
        Bit#(8) idx = save_addressQ.first;

        Bit#(32) base = fromInteger(base_idx) + (1024 * 1024 * 2 * zeroExtend(idx));

        Bit#(32) target_addr = d * 2048 + base;

        // flush to Second_sorter , barrier will be needed in future
        
        if (d== 1023) begin // first 2MB
            dram_fullQ.enq(base); // start with 0 idx
            $display("full req from %d", idx);
        end        
        radix_dram_reqQ.enq(tuple4(target_addr , 1, 1, 4));
        bram_ctl.write_req(idx, truncate(d + 1));
    endrule

    rule putToDRAMarbiter;
        temp_save_to_firstOutQ.deq;
        let d = temp_save_to_firstOutQ.first;

        dramArbiter.put[4].put(d);
    endrule

    Reg#(Bit#(32)) sort_dram_r_req <- mkReg(0);
    Reg#(Bit#(32)) sort_dram_addr <- mkReg(0);
    FIFO#(Bit#(512)) save_first_sortedQ <- mkSizedBRAMFIFO(1024);
    Reg#(Bit#(32)) read_done_cnt <- mkReg(0);
    Reg#(Bit#(5)) read_done_temp_cnt <- mkReg(0);
    Reg#(Bit#(32)) read_req_cnt <- mkReg(0);

    rule radix_dramFlush_req(sort_dram_r_req == 0);
        dram_fullQ.deq;
        Bit#(32) target_addr = dram_fullQ.first;
        sort_dram_addr <= target_addr;
        sort_dram_r_req <= 1024;
    endrule

(* descending_urgency = "calculate_DRAM_address, dram_read_req_sorted_data" *)
    rule dram_read_req_sorted_data(sort_dram_r_req != 0 && read_req_cnt - read_done_cnt < 4);
        /* $display("Second req!! target : %d current : %d ",sort_dram_r_req, read_req_cnt); */
        sort_dram_r_req <= sort_dram_r_req - 1;
        read_req_cnt <= read_req_cnt + 1;
        radix_dram_reqQ.enq(tuple4(sort_dram_addr + 2048 * (sort_dram_r_req - 1), 0, 1, 4)); //2KB read/write
    endrule

    rule radix_command;
        radix_dram_reqQ.deq;
        let d = radix_dram_reqQ.first;
        
        dramArbiter.req(d);
    endrule

    rule get_data_from_arbiter;
        let d <- dramArbiter.get[4].get; // 16B input
        save_first_sortedQ.enq(d);
    endrule

    rule put_radix_second_serial;
        save_first_sortedQ.deq;
        let d = save_first_sortedQ.first;
        serial_secondsQ.put(d);

        read_done_temp_cnt <= read_done_temp_cnt + 1;
        if (read_done_temp_cnt == 0) begin // 2KB read starts
            read_done_cnt <= read_done_cnt + 1;
        end
    endrule

    rule put_radix_second;
        let d <- serial_secondsQ.get;
        Vector#(4,Bit#(32)) ind;
        ind[0] = d[127:96];
        ind[1] = d[95:64];
        ind[2] = d[63:32];
        ind[3] = d[31:0];
        radixSecond.enq(ind);
    endrule

    rule get_data_from_radix_second;
        radixSecond.deq;
        let d = radixSecond.first;
        toUploader.enq(d);
    endrule

    Vector#(2, FIFO#(Bit#(128))) to_semisorterQ <- replicateM(mkFIFO);
    Vector#(2, FIFO#(Bit#(128))) toNodeQ <- replicateM(mkFIFO);
    Vector#(2, Reg#(Bit#(128))) node_cache <- replicateM(mkReg(0));
    Reg#(Bit#(2)) node_rullet <- mkReg(0);

    FIFO#(Bit#(128)) save_to_uploaderQ <- mkFIFO;

    rule raidx_to_node_bridge;
        toUploader.deq;
        Vector#(4, Bit#(32)) t = toUploader.first;
        Bit#(128) d = 0;
        for (Bit#(4) i = 0; i < 4; i = i + 1) begin
            d = d << 32;
            d = d | zeroExtend(t[i]);
        end
        
        for (Bit#(4) i = 0; i < 2; i = i + 1) begin
            to_semisorterQ[i].enq(d);
        end
    endrule

    Vector#(2, SerializerIfc#(128, 2)) serial_nodeQ <- replicateM(mkSerializer);
    for (Bit#(4) i = 0; i < 2; i = i + 1) begin
        rule semi_sorting_rules;
            to_semisorterQ[i].deq;
            let d = to_semisorterQ[i].first;
            Vector#(4,Bit#(32)) ind;
            ind[0] = d[127:96];
            ind[1] = d[95:64];
            ind[2] = d[63:32];
            ind[3] = d[31:0];
            Bit#(128) r_data = 0 ;

            for (Bit#(4) j = 0; j < 4; j = j + 1) begin
                Bit#(1) id = 0;
                Bit#(16) t = truncate(ind[i]);
                id = truncateLSB(t);
                if (id == truncate(i)) begin
                    r_data = r_data << 32;
                    r_data = r_data | zeroExtend(ind[i] | 255);
                end
            end
            if (r_data != 0) begin
                serial_nodeQ[i].put(r_data);
            end
        endrule
    end

/////////////////////////// Node uploader part ////////////

    Reg#(Bit#(1)) rullet <- mkReg(0);
    Reg#(Bit#(32)) req_counter <- mkReg(0);

    Reg#(Bit#(1)) req_merger <- mkReg(0);

    for (Bit#(4) i = 0; i < 2; i = i + 1) begin
        rule put_data_to_node;
            let d <- serial_nodeQ[i].get;
            if (d != 0) begin
                node[i].enq(d);
            end
        endrule
    end

    Vector#(2, FIFO#(Tuple4#(Bit#(32), Bit#(1), Bit#(16), Bit#(3)))) dram_reqQ <- replicateM(mkFIFO);

    rule rullet_rule(start_cnt == 1);
        rullet <= rullet + 1;
        clock_cnt <= clock_cnt + 1;
        node_rullet <= node_rullet + 1;
    endrule

    rule get_req;
        let write_addr <- node[rullet].dram_write_req;
        let read_addr <- node[rullet].dram_read_req;
        //req_counter <= req_counter + 1;
        $display("req counting is %d ", rullet);

        dram_reqQ[0].enq(tuple4(zeroExtend(write_addr), 1, 4, zeroExtend(rullet)));
        dram_reqQ[1].enq(tuple4(zeroExtend(read_addr), 0, 4, zeroExtend(rullet)));
    endrule

    Vector#(2, FIFO#(Bit#(512))) bloom_dram_outQ <- replicateM(mkSizedBRAMFIFO(256));
    Vector#(2, FIFO#(Bit#(512))) bloom_dram_inQ <- replicateM(mkSizedBRAMFIFO(256));

(* descending_urgency = "put_req, radix_command" *)
    rule put_req;
        dram_reqQ[req_merger].deq;
        dramArbiter.req(dram_reqQ[req_merger].first);
        req_merger <= req_merger + 1;
    endrule

    for (Bit#(4) i = 0; i < 2; i = i + 1) begin
        rule putDRAMsave_deserial;
            let d <- node[i].dram_write_data_get;
            bloom_dram_outQ[i].enq(d);
        endrule

        rule putDRAMdata;
            bloom_dram_outQ[i].deq;
            let d = bloom_dram_outQ[i].first;
            dramArbiter.put[i].put(d);
        endrule

        rule getDRAMdata_save;
            let d <- dramArbiter.get[i].get;
            bloom_dram_inQ[i].enq(d);
        endrule

        rule getDRAMdata;
            bloom_dram_inQ[i].deq;
            let d = bloom_dram_inQ[i].first;
            node[i].dram_enq(d);
        endrule
    end
////////////End of Arbiter //////////////////


    /* Giving DMA write done signal to the HOST */
    rule getSignalFromHost;
        let r <- pcie.dataReq;
        let a = r.addr;
        let offset = (a>>2);
        if (offset != 0) begin
            pcie_write_doneQ.enq(tuple2(r, 1));
        end
    endrule
    rule sendDoneSignalToHost;
        pcie_write_doneQ.deq;
        let r = tpl_1(pcie_write_doneQ.first);
        let offset = tpl_2(pcie_write_doneQ.first);
        if (offset == 1) begin
            dmaWriteDoneSignalQ.deq;
            pcie.dataSend(r, 1);
        end
    endrule
endmodule
