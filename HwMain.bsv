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

    BloomNodeIfc node <- mkBloomNode;

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

/*             if (i == 0) begin
 *                 if (temp_val % 2048 == 0) begin
 *                     temp_val <= temp_val + (1 << 16) + 1;
 *                     $display("good !!");
 *                 end else begin
 *                     temp_val <= temp_val + 1;
 *                 end
 *             end
 *
 *             d1 = temp_val;
 *             d2 = temp_val; */

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

    // prepatcher - applyer start

    Reg#(Bit#(1)) dram_req_handle <- mkReg(0);

    Reg#(Bit#(32)) dram_write_idx <- mkReg(0);
    Reg#(Bit#(32)) dram_write_cnt <- mkReg(128);
    Reg#(Bit#(32)) dram_read_idx <- mkReg(0);
    Reg#(Bit#(32)) dram_read_cnt <- mkReg(128);
    Reg#(Bit#(32)) dram_read_put_cnt <- mkReg(128);

    rule generate_bit_data;
        Bit#(32) t <- serial_to_radixQ.get;
        node.enq(t);
    endrule

    rule write_dram_req(dram_req_handle == 0 && dram_write_cnt == 128);
        let t <- node.dram_write_req;
        Bit#(32) d = zeroExtend(t);
        dram_write_idx <= d * 1024 * 8;
        dram_write_cnt <= 0;
    endrule

    rule write_dram_data(dram_write_cnt < 128 && dram_req_handle == 0);
        Bit#(512) d <- node.dram_write_data_get;
        Bit#(32) idx = dram_write_idx + (dram_write_cnt * 64);
        dram.write(zeroExtend(idx), d , 64);
        $display("dram_write req");
        dram_write_cnt <= dram_write_cnt + 1;

        if (dram_write_cnt == 127) begin
            $display("read to write!!");
            dram_req_handle <= 1;
        end
    endrule

    rule read_dram_req(dram_req_handle == 1 && dram_read_cnt == 128);
        let t <- node.dram_read_req;
        Bit#(32) d = zeroExtend(t);
        dram_read_idx <= d * 1024 * 8;
        dram_read_cnt <= 0;
        dram_read_put_cnt <= 0;
    endrule

    rule read_dram_data(dram_req_handle == 1 && dram_read_cnt < 128);
        Bit#(32) idx = dram_read_idx + (dram_read_cnt * 64);
        $display("dram_read req");
        dram.readReq(zeroExtend(idx), 64);
        dram_read_cnt <= dram_read_cnt + 1;
    endrule

    rule read_dram_put(dram_req_handle == 1 && dram_read_put_cnt < 128);
        dram_read_put_cnt <= dram_read_put_cnt + 1;
        if (dram_read_put_cnt == 127) begin
            dram_req_handle <= 0;
            $display("write to read!!");
        end
        Bit#(512) d <- dram.read;
        node.dram_enq(d);
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
