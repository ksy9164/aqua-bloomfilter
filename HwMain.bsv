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

interface HwMainIfc;
endinterface

module mkHwMain#(PcieUserIfc pcie, DRAMUserIfc dram) 
    (HwMainIfc);
    Reg#(Bit#(32)) file_size <- mkReg(2550000);
    Reg#(Bit#(32)) dramWriteCnt <- mkReg(0);
    Reg#(Bit#(32)) dramReadCnt <- mkReg(0);
    FIFOLI#(Tuple2#(Bit#(20), Bit#(32)), 3) pcie_reqQ <- mkFIFOLI;

    DeSerializerIfc#(128, 4) deserial_dram <- mkDeSerializer;
    SerializerIfc#(128, 2) serial_dmaQ <- mkSerializer;
    SerializerIfc#(256, 8) serial_to_radixQ <- mkSerializer;

    FIFOLI#(Bit#(32), 2) dmaReadReqQ <- mkFIFOLI;
    FIFOLI#(Bit#(32), 2) dmaWriteReqQ <- mkFIFOLI;
    FIFOLI#(Bit#(1), 2) dmaWriteDoneSignalQ <- mkFIFOLI;

    Reg#(Bit#(32)) readCnt <- mkReg(0);

    FIFO#(Bit#(64)) kmerQ <- mkFIFO;
    Reg#(Bit#(64)) kmer64bits <- mkReg(0);
    Reg#(Bit#(6)) kmerCnt <- mkReg(0);

    /* Vector#(2, Vector#(8, BloomHashIfc)) hashfuncQ <- replicateM(replicateM(mkJsHash)); */

    FIFOLI#(Tuple2#(PcieCtrl::IOReadReq, Bit#(1)), 3) pcie_write_doneQ <- mkFIFOLI;
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

    Vector#(2, FIFOLI#(Bit#(64), 3)) spread_inputQ <- replicateM(mkFIFOLI);

    Vector#(4, FIFO#(Bit#(64))) merge_st1Q <- replicateM(mkFIFO);
    Vector#(2, FIFO#(Bit#(128))) merge_st2Q <- replicateM(mkFIFO);
    FIFO#(Bit#(512)) toDramQ <- mkFIFO;

    Vector#(2, Reg#(Bit#(64))) merge_buf_1 <- replicateM(mkReg(0)); 
    Vector#(2, Reg#(Bit#(1))) merge_buf_1_ctl <- replicateM(mkReg(0)); 

    FIFO#(Bit#(128)) hashedDataQ <- mkFIFO;

    Reg#(Bit#(128)) merge_buf_2 <- mkReg(0); 
    Reg#(Bit#(1)) merge_buf_2_ctl <- mkReg(0); 
    rule getDataFromHost;
        let w <- pcie.dataReceive;
        let a = w.addr;
        let d = w.data;
        pcie_reqQ.enq(tuple2(a, d));
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

    for (Bit#(8) i = 0; i < 8; i = i + 1) begin
        rule hashing;
            Bit#(64) kmer <- toHashMultiQ.get[i].get;
            hashfuncQ[i].enq(kmer);
        endrule
    end

//// TODO

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

/*     rule temp_g;
 *         Bit#(32) t <- serial_to_radixQ.get;
 *     endrule
 *
 *     FIFO#(Bit#(32)) tempQ <- mkFIFO;
 *     Reg#(Bit#(32)) temp_cnt <- mkReg(0);
 *     Reg#(Bit#(64)) real_cnt <- mkReg(0);
 *     Reg#(Bit#(64)) clock_cnt <- mkReg(0);
 *     rule clock_counter;
 *         $display("clock : %d data : %d", clock_cnt, real_cnt);
 *         clock_cnt <= clock_cnt + 1;
 *     endrule
 *
 *     rule temp;
 *         tempQ.enq(temp_cnt);
 *         if (temp_cnt % 2048 == 0) begin
 *             temp_cnt <= temp_cnt + 65536 + 1;
 *             $display("chacha @");
 *         end else begin
 *             temp_cnt <= temp_cnt + 1;
 *         end
 *         real_cnt <= real_cnt + 1;
 *     endrule */

    FIFO#(Bit#(128)) bram_dataQ <- mkFIFO;
    FIFO#(Bit#(16)) dram_idxQ <- mkFIFO;
    FIFO#(Bit#(9)) bram_idxQ <- mkFIFO;
    rule generate_bit_data;
        Bit#(32) t <- serial_to_radixQ.get;
        /* tempQ.deq;
         * Bit#(32) t = tempQ.first; */

        Bit#(16) dram_idx = truncateLSB(t);

        Bit#(16) data = truncate(t);
        Bit#(7)  bram_data = truncate(data);
        Bit#(9)  bram_idx = truncateLSB(data);

        Bit#(128) write_d = 0;
        write_d[bram_data] = 1;

        bram_dataQ.enq(write_d);
        bram_idxQ.enq(bram_idx);
        dram_idxQ.enq(dram_idx);
    endrule

    Vector#(2, BramCtlIfc#(128, 512, 9)) bram_ctl <- replicateM(mkBramCtl);
    Vector#(2, Reg#(Bit#(1))) bram_condition <- replicateM(mkReg(0));
    Vector#(2, Reg#(Bit#(16))) dram_target_idx <- replicateM(mkReg(0));

    Vector#(2, FIFO#(Bit#(128))) toBramctl_dataQ <- replicateM(mkFIFO);
    Vector#(2, FIFO#(Bit#(9))) toBramctl_idxQ <- replicateM(mkFIFO);
    Vector#(2, FIFO#(Bit#(9))) toBram_writer_idxQ <- replicateM(mkFIFO);
    Vector#(2, FIFOLI#(Bit#(16), 3)) toDramloaderQ <- replicateM(mkFIFOLI);
    Reg#(Bit#(1)) bram_handle <- mkReg(0);

    rule bram_loader(bram_condition[bram_handle] == 0); // to avoid DRAM conflict
        bram_dataQ.deq;
        bram_idxQ.deq;
        dram_idxQ.deq;
        Bit#(16) dram_idx = dram_idxQ.first;
        Bit#(1) idx = bram_handle;
        if (dram_idx != dram_target_idx[bram_handle]) begin
            toBramctl_idxQ[idx + 1].enq(bram_idxQ.first);
            toBramctl_dataQ[idx + 1].enq(bram_dataQ.first);

            toDramloaderQ[idx].enq(dram_target_idx[bram_handle]);
            bram_condition[idx] <= 1;

            dram_target_idx[bram_handle + 1] <= dram_idx; 
            bram_handle <= bram_handle + 1;
        end else begin
            toBramctl_idxQ[idx].enq(bram_idxQ.first);
            toBramctl_dataQ[idx].enq(bram_dataQ.first);
        end
    endrule

    for (Bit#(8) i = 0; i < 2; i = i + 1) begin
        rule bram_reader(bram_condition[i] == 0);
            toBramctl_idxQ[i].deq;
            Bit#(9) bram_idx = toBramctl_idxQ[i].first;
            toBram_writer_idxQ[i].enq(bram_idx);
            bram_ctl[i].read_req(bram_idx);
        endrule

        rule bram_writer;
            Bit#(128) d <- bram_ctl[i].get;
            Bit#(128) write_data = toBramctl_dataQ[i].first;
            toBram_writer_idxQ[i].deq;
            toBramctl_dataQ[i].deq;
            d = d | write_data;
            bram_ctl[i].write_req(toBram_writer_idxQ[i].first, d);
        endrule
    end

    Reg#(Bit#(3)) dram_step <- mkReg(0);
    Reg#(Bit#(1)) target_bram_to_dram <- mkReg(0);
    Reg#(Bit#(32)) dram_target <- mkReg(0);

    rule bram_applier(bram_condition[target_bram_to_dram] == 1 && dram_step == 0);
        toDramloaderQ[target_bram_to_dram].deq;
        dram_target <= zeroExtend(toDramloaderQ[target_bram_to_dram].first) * 1024 * 8;
        dram_step <= 1;
    endrule

    Reg#(Bit#(9)) bram_cnt <- mkReg(0);
    Reg#(Bit#(32)) dram_read_cnt <- mkReg(0);
    Reg#(Bit#(32)) dram_comb_cnt <- mkReg(0);
    Reg#(Bit#(32)) dram_write_cnt <- mkReg(0);
    FIFO#(Bit#(512)) dram_combinedDataQ <- mkSizedBRAMFIFO(128);
    FIFO#(Bit#(128)) bram_combinedDataQ <- mkSizedBRAMFIFO(512);
    rule read_data_from_bram_for_dram(dram_step == 1);
        Bit#(1) bram_target = target_bram_to_dram;
        bram_ctl[bram_target].read_req(bram_cnt);
        bram_cnt <= bram_cnt + 1;
        if (bram_cnt == 511) begin
            dram_step <= 2;
        end
    endrule

    rule deserial_to_dram;
        Bit#(1) bram_target = target_bram_to_dram;
        Bit#(128) d <- bram_ctl[bram_target].get;
        bram_combinedDataQ.enq(d);
    endrule

    rule deserial_bram;
        bram_combinedDataQ.deq;
        let d = bram_combinedDataQ.first;
        deserial_dram.put(d);
    endrule

    rule read_req_to_dram(dram_step == 2);
        Bit#(32) idx = dram_target + (dram_read_cnt * 64);
        dram.readReq(zeroExtend(idx), 64);
        if (dram_read_cnt == 127) begin
            dram_read_cnt <= 0;
            dram_step <= 3;
        end else begin
            dram_read_cnt <= dram_read_cnt + 1;
        end
    endrule

    rule read_dram_and_pass(dram_step == 2 || dram_step == 3);
        let d <- deserial_dram.get;
        Bit#(512) dram_data <- dram.read;
        d = d | dram_data;
        if (dram_comb_cnt == 127 && dram_step == 3) begin
            dram_comb_cnt <= 0;
            dram_step <= 4;
        end else begin
            dram_comb_cnt <= dram_comb_cnt + 1;
        end
        dram_combinedDataQ.enq(d);
    endrule

    rule write_req_to_dram(dram_step == 4);
        dram_combinedDataQ.deq;
        Bit#(512) d = dram_combinedDataQ.first;

        Bit#(32) idx = dram_target + (dram_write_cnt * 64);
        dram.write(zeroExtend(idx), d , 64);
        if (dram_write_cnt == 127) begin
            dram_step <= 5;
            dram_write_cnt <= 0;
        end else begin
            dram_write_cnt <= dram_write_cnt + 1;
        end
        $display("gogo !!");
    endrule

    Reg#(Bit#(9)) bram_init_cnt <- mkReg(0);
    rule init_bram_data(dram_step == 5);
        bram_ctl[target_bram_to_dram].write_req(bram_init_cnt, 0);
        bram_init_cnt <= bram_init_cnt + 1;
        if (bram_init_cnt == 511) begin
            dram_step <= 0;
            bram_condition[target_bram_to_dram] <= 0; // all is done
            target_bram_to_dram <= target_bram_to_dram + 1;
        end
    endrule

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
