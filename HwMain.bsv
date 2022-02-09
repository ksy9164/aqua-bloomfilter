import FIFO::*;
import FIFOF::*;
import Clocks::*;
import Vector::*;
import BRAM::*;
import BRAMFIFO::*;

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

    SerializerIfc#(512, 4) serial_dramQ <- mkSerializer;
    SerializerIfc#(512, 16) serial_inputQ <- mkSerializer;

    FIFOLI#(Bit#(32), 2) dmaReadReqQ <- mkFIFOLI;
    FIFOLI#(Bit#(32), 2) dmaWriteReqQ <- mkFIFOLI;
    FIFOLI#(Bit#(1), 2) dramWriteDoneSignalQ <- mkFIFOLI;
    FIFOLI#(Bit#(1), 2) dmaWriteDoneSignalQ <- mkFIFOLI;

    Reg#(Bit#(32)) readCnt <- mkReg(0);

    Reg#(Bit#(1)) dmaWriteHandle <- mkReg(0);
    Reg#(Bit#(32)) dmaWriteTarget <- mkReg(0);
    Reg#(Bit#(32)) dmaWriteCnt <- mkReg(0);

    FIFO#(Bit#(64)) kmerQ <- mkFIFO;
    SerializerIfc#(128, 2) inputSerialQ <- mkSerializer;
    FIFO#(Bit#(64)) kmerGenQ <- mkFIFO;
    Reg#(Bit#(64)) kmer64bits <- mkReg(0);
    Reg#(Bit#(6)) kmerCnt <- mkReg(0);

    Vector#(2 , MultiOneToEightIfc#(Bit#(64))) toHashMultiQ <- replicateM(mkMultiOnetoEight);

    FIFOLI#(Tuple2#(PcieCtrl::IOReadReq, Bit#(1)), 2) pcie_write_doneQ <- mkFIFOLI;

    Vector#(2, Vector#(8, BloomHashIfc)) hashfuncQ <- replicateM(replicateM(mkJsHash));
    hashfuncQ[0][0] <- mkJsHash;
    hashfuncQ[0][1] <- mkBernsteinHash;
    hashfuncQ[0][2] <- mkAddictiveHash;
    hashfuncQ[0][3] <- mkRotatingHash;
    hashfuncQ[0][4] <- mkBkdrHash;
    hashfuncQ[0][5] <- mkSdbmHash;
    hashfuncQ[0][6] <- mkDjbHash;
    hashfuncQ[0][7] <- mkDekHash;

    hashfuncQ[1][0] <- mkJsHash;
    hashfuncQ[1][1] <- mkAddictiveHash;
    hashfuncQ[1][2] <- mkSdbmHash;
    hashfuncQ[1][3] <- mkDjbHash;
    hashfuncQ[1][4] <- mkBernsteinHash;
    hashfuncQ[1][5] <- mkBkdrHash;
    hashfuncQ[1][6] <- mkDekHash;
    hashfuncQ[1][7] <- mkRotatingHash;

    Vector#(2, FIFOLI#(Bit#(64), 3)) spread_inputQ <- replicateM(mkFIFOLI);

    Vector#(8, FIFO#(Bit#(64))) merge_st1Q <- replicateM(mkFIFO);
    Vector#(4, FIFO#(Bit#(128))) merge_st2Q <- replicateM(mkFIFO);
    Vector#(2, FIFO#(Bit#(256))) merge_st3Q <- replicateM(mkFIFO);
    FIFO#(Bit#(512)) toDramQ <- mkSizedBRAMFIFO(20000000);
    DeSerializerIfc#(256, 2) deserial_dram <- mkDeSerializer;

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
        end else if (off == 2) begin
            dmaWriteReqQ.enq(d);
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

        inputSerialQ.put(rd);
        /* spread_inputQ[0].enq(truncate(rd));
         * spread_inputQ[1].enq(truncateLSB(rd)); */
    endrule

    Reg#(Bit#(128)) kmer_buffer <- mkReg(0);
    Reg#(Bit#(7)) kmer_buffer_idx <- mkReg(0);
    rule kmerGenerate;
        Bit#(128) t_buf = kmer_buffer;
        Bit#(7) idx = kmer_buffer_idx;
        if (kmer_buffer_idx == 62) begin
            Bit#(64) t <- inputSerialQ.get;
            Bit#(128) d = zeroExtend(t);
            d = d << 62;
            t_buf = t_buf | d;
            idx = idx + 64;
        end else if (kmer_buffer_idx == 0) begin
            Bit#(64) t <- inputSerialQ.get;
            Bit#(128) d = zeroExtend(t);
            t_buf = t_buf | d;
            idx = idx + 64;
        end
        spread_inputQ[0].enq(truncate(t_buf));
        kmer_buffer <= (t_buf >> 2);
        kmer_buffer_idx <= idx - 2;
    endrule

    rule distributeKmer;
        spread_inputQ[0].deq;
        toHashMultiQ[0].enq(spread_inputQ[0].first);
    endrule

    for (Bit#(8) j = 0; j < 8; j = j + 1) begin
        rule hashing;
            Bit#(64) kmer <- toHashMultiQ[0].get[j].get;
            hashfuncQ[0][j].enq(kmer);
        endrule
    end

    /* Merging */
    for (Bit#(8) i = 0; i < 4; i = i + 1) begin
        rule hashGet_merge_step_1;
            Bit#(32) d1 <- hashfuncQ[0][i * 2].get;
            Bit#(32) d2 <- hashfuncQ[0][i * 2 + 1].get;

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
    rule hashGet_merge_step_4;
        merge_st2Q[0].deq;
        merge_st2Q[1].deq;
        Bit#(256) d1 = zeroExtend(merge_st2Q[0].first);
        Bit#(256) d2 = zeroExtend(merge_st2Q[1].first);
        d2 = d2 << 128;

        deserial_dram.put(d1 | d2);
    endrule
    Reg#(Bit#(32)) dramCnt <- mkReg(0);

    rule toDramWrite(dramCnt != 25000000 / 16);
        let d <- deserial_dram.get;
        toDramQ.enq(d);
        dramCnt <= dramCnt + 1;
    endrule

    // Testing version start
    Reg#(Bit#(32)) dataAmountCnt <- mkReg(0);
    Reg#(Bit#(32)) hitCnt <- mkReg(0);
    Reg#(Bit#(32)) missCnt <- mkReg(0);
    Reg#(Bit#(64)) clockCnt <- mkReg(0);

    Reg#(Bit#(512)) dramBuff <- mkReg(0);
    Reg#(Bit#(23)) idxMap <- mkReg(0);

    rule send_data_32bits(dramCnt == 25000000 / 16);
        toDramQ.deq;
        serial_inputQ.put(toDramQ.first);
    endrule

    rule clockCounter(dramCnt == 25000000 / 16);
        clockCnt <= clockCnt + 1;
    endrule

    FIFOLI#(Bit#(512), 50) dramSendQ <- mkFIFOLI;
    FIFOLI#(Bit#(23), 50) dramIdxQ <- mkFIFOLI;
    rule writeDRAM;
        dramSendQ.deq;
        dramIdxQ.deq;
        Bit#(512) d <- dram.read;
        d = d | dramSendQ.first;
        dram.write(zeroExtend(dramIdxQ.first), d, 64);
    endrule

    rule get_data_32bits;
        Bit#(32) data <- serial_inputQ.get;
        Bit#(23) idx = truncateLSB(data);
        Bit#(9) mark_data = truncate(data);

        Bit#(512) target_hit = 1;
        target_hit = target_hit << mark_data;

        if (idx != idxMap) begin
            dramBuff <= target_hit; 
            missCnt <= missCnt + 1;
            dram.readReq(zeroExtend(idxMap), 64);
            dramSendQ.enq(dramBuff);
            dramIdxQ.enq(idxMap);
            idxMap <= idx;
        end else begin
            dramBuff <= dramBuff | target_hit;
            hitCnt <= hitCnt + 1;
        end
        dataAmountCnt <= dataAmountCnt + 1;

        if (dataAmountCnt % 10000 == 0) begin
            $display("total %d | hit %d | miss %d | percent %d | clock %d", dataAmountCnt, hitCnt, missCnt, (hitCnt * 100) / (dataAmountCnt + 1), clockCnt);
        end
    endrule

/*
    rule dramWrite(dramWriteCnt < file_size - 64); // have to fix it for not losing last 64bytes
        dramWriteCnt <= dramWriteCnt + 64;
        toDramQ.deq;
        Bit#(512) d = toDramQ.first;
        $display("current : %d || target : ", dramWriteCnt, file_size);
        $display("%b ", d);
        dram.write(zeroExtend(dramWriteCnt), d, 64);
    endrule

    rule dramReadReq(dramWriteCnt >= file_size - 64 && dramReadCnt < file_size);
        dramReadCnt <= dramReadCnt + 64;
        dram.readReq(zeroExtend(dramReadCnt), 64);
    endrule
    rule dramRead;
        Bit#(512) d <- dram.read;
        serial_dramQ.put(d);
    endrule

    rule getDmaWriteReq(dmaWriteHandle == 0);
        dmaWriteReqQ.deq;
        pcie.dmaWriteReq(0, truncate(dmaWriteReqQ.first));
        dmaWriteHandle <= 1;
        dmaWriteTarget <= dmaWriteReqQ.first;
        dmaWriteCnt <= 0;
    endrule
    rule putDataToDma(dmaWriteHandle != 0);
        Bit#(128) d <- serial_dramQ.get;
        pcie.dmaWriteData(d);
        if (dmaWriteCnt + 1 == dmaWriteTarget) begin
            dmaWriteHandle <= 0;
            dramWriteDoneSignalQ.enq(1);
        end else begin
            dmaWriteCnt <= dmaWriteCnt + 1;
        end
    endrule
*/

        /* Giving DMA write done signal to the HOST */
    rule getSignalFromHost;
        let r <- pcie.dataReq;
        let a = r.addr;
        let offset = (a>>2);
        if (offset == 0) begin
            pcie_write_doneQ.enq(tuple2(r, 0));
        end else begin
            pcie_write_doneQ.enq(tuple2(r, 1));
        end
    endrule
    rule sendDoneSignalToHost;
        pcie_write_doneQ.deq;
        let r = tpl_1(pcie_write_doneQ.first);
        let offset = tpl_2(pcie_write_doneQ.first);
        if (offset == 0) begin
            dramWriteDoneSignalQ.deq;
            $display("dram write done!");
            pcie.dataSend(r, 1);
        end else begin
            dmaWriteDoneSignalQ.deq;
            $display("dma write done!");
            pcie.dataSend(r, 1);
        end
    endrule
endmodule
