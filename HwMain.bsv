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
    Reg#(Bit#(32)) file_size <- mkReg(0);
    Reg#(Bit#(32)) dramWriteCnt <- mkReg(0);
    Reg#(Bit#(32)) dramReadCnt <- mkReg(0);
    FIFOLI#(Tuple2#(Bit#(20), Bit#(32)), 2) pcie_reqQ <- mkFIFOLI;

    SerializerIfc#(128, 4) serial_pcieio <- mkSerializer;
    SerializerIfc#(128, 16) serial_input <- mkSerializer;
    DeSerializerIfc#(128, 4) deserial_dram <- mkDeSerializer;
    SerializerIfc#(512, 4) serial_dramQ <- mkSerializer;

    FIFO#(Bit#(32)) dmaReadReqQ <- mkFIFO;
    FIFO#(Bit#(32)) dmaWriteReqQ <- mkFIFO;
    FIFO#(Bit#(1)) dramWriteDoneSignalQ <- mkFIFO;
    FIFO#(Bit#(1)) dmaWriteDoneSignalQ <- mkFIFO;

    Reg#(Bit#(32)) readCnt <- mkReg(0);

    Reg#(Bit#(1)) dmaWriteHandle <- mkReg(0);
    Reg#(Bit#(32)) dmaWriteTarget <- mkReg(0);
    Reg#(Bit#(32)) dmaWriteCnt <- mkReg(0);

    FIFO#(Bit#(2)) encodedDataQ <- mkFIFO;
    FIFO#(Bit#(64)) kmerQ <- mkFIFO;
    Reg#(Bit#(64)) kmer64bits <- mkReg(0);
    Reg#(Bit#(6)) kmerCnt <- mkReg(0);

    MultiOneToFourIfc#(Bit#(64)) toHashMultiQ <- mkMultiOnetoFour;

    Vector#(4, BloomHashIfc) hashfuncQ;
    hashfuncQ[0] <- mkJshash;
    hashfuncQ[1] <- mkElfhash;
    hashfuncQ[2] <- mkSdbmhash;
    hashfuncQ[3] <- mkDjbhash;

    Vector#(4, FIFOF#(Bit#(32))) merge_st1Q <- replicateM(mkFIFOF);
    Vector#(2, FIFOF#(Bit#(64))) merge_st2Q <- replicateM(mkFIFOF);
    Vector#(2, FIFOF#(Bit#(64))) hashedMergeDataQ <- replicateM(mkFIFOF);

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
        if ( off == 0 ) begin
            file_size <= d;
        end else if (off == 1) begin // Log Data In
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
        serial_input.put(rd);
    endrule

    rule twoBitsEncoding;
        Bit#(8) d <- serial_input.get;
        case (d)
            65: encodedDataQ.enq(0);
            97: encodedDataQ.enq(0);
            67: encodedDataQ.enq(1);
            99: encodedDataQ.enq(1);
            71: encodedDataQ.enq(2);
            103: encodedDataQ.enq(2);
            84: encodedDataQ.enq(3);
            116: encodedDataQ.enq(3);
        endcase
    endrule

    rule make32BitsKmerChunks;
        encodedDataQ.deq;
        Bit#(64) d = zeroExtend(encodedDataQ.first);
        Bit#(64) kmer = kmer64bits;
        kmer = kmer | d;
        if (kmerCnt < 30) begin
            kmerCnt <= kmerCnt + 1;
        end else begin
            kmerQ.enq(kmer);
        end
        kmer64bits <= kmer << 2;
    endrule

    rule distributeKmer;
        kmerQ.deq;
        toHashMultiQ.enq(kmerQ.first);
    endrule

    for (Bit#(8) i = 0; i < 4; i = i + 1) begin
        rule hashing;
            Bit#(64) kmer <- toHashMultiQ.get[i].get;
            if (i == 0)
                hashfuncQ[i].enq(kmer);
        endrule
    end

    Bit#(32) query = 32'h00000001; // just for a test
    for (Bit#(8) i = 0; i < 4; i = i + 1) begin
        rule hashGet;
            Bit#(64) d <- hashfuncQ[i].get;
            Bit#(32) lower = truncate(d);
            Bit#(32) upper = truncate(d >> 32);
            Bit#(32) mask = 3;
            if ((mask & upper) == query) begin
                merge_st1Q[i].enq(lower);
            end
        endrule
    end

    for (Bit#(3) i = 0; i < 2; i = i + 1) begin
        rule merge_step1;
            Bit#(3) id_one = i * 2;
            Bit#(3) id_two = i * 2 + 1;
            Bit#(64) d = 0;

            if (merge_st1Q[id_one].notEmpty && merge_st1Q[id_two].notEmpty) begin
                merge_st1Q[id_one].deq;
                merge_st1Q[id_two].deq;
                Bit#(64) d1 = zeroExtend(merge_st1Q[id_one].first);
                Bit#(64) d2 = zeroExtend(merge_st1Q[id_two].first);
                d1 = d1 << 32;
                d = d1 | d2;
                merge_st2Q[i].enq(d);
            end else begin
                if (merge_st1Q[id_one].notEmpty) begin
                    merge_st1Q[id_one].deq;
                    d = zeroExtend(merge_st1Q[id_one].first);
                end else begin
                    merge_st1Q[id_two].deq;
                    d = zeroExtend(merge_st1Q[id_two].first);
                end

                if (merge_buf_1_ctl[i] == 1) begin // buf is not empty
                    Bit#(64) buf_d = merge_buf_1[i];
                    merge_st2Q[i].enq(d | buf_d);
                    merge_buf_1[i] <= 0;
                    merge_buf_1_ctl[i] <= 0;
                end else begin
                    merge_buf_1[i] <= (d << 32);
                    merge_buf_1_ctl[i] <= 1;
                end
            end
        endrule
    end

    rule merge_step2;
        Bit#(128) d = 0;

        if (merge_st2Q[0].notEmpty && merge_st2Q[1].notEmpty) begin
            merge_st2Q[0].deq;
            merge_st2Q[1].deq;
            Bit#(128) d1 = zeroExtend(merge_st2Q[0].first);
            Bit#(128) d2 = zeroExtend(merge_st2Q[1].first);
            d1 = d1 << 64;
            d = d1 | d2;
            hashedDataQ.enq(d);
        end else begin
            if (merge_st2Q[0].notEmpty) begin
                merge_st2Q[0].deq;
                d = zeroExtend(merge_st2Q[0].first);
            end else begin
                merge_st2Q[1].deq;
                d = zeroExtend(merge_st2Q[1].first);
            end

            if (merge_buf_2_ctl == 1) begin // buf is not empty
                hashedDataQ.enq(d | merge_buf_2);
                merge_buf_2 <= 0;
                merge_buf_2_ctl <= 0;
            end else begin
                merge_buf_2 <= (d << 64);
                merge_buf_2_ctl <= 1;
            end
        end
    endrule

    rule toDram;
        hashedDataQ.deq;
        $display("%d ", hashedDataQ.first);
        deserial_dram.put(hashedDataQ.first);
    endrule

    /* Write to DRAM */
    rule dramWrite(dramWriteCnt < file_size - 64); // have to fix it for not losing last 64bytes
        dramWriteCnt <= dramWriteCnt + 64;
        Bit#(512) d <- deserial_dram.get;
        dram.write(zeroExtend(dramWriteCnt), d, 64);
    endrule

    /* DRAM read */
    rule dramReadReq(dramWriteCnt >= file_size - 64 && dramReadCnt < file_size);
        dramReadCnt <= dramReadCnt + 64;
        dram.readReq(zeroExtend(dramReadCnt), 64);
    endrule
    rule dramRead;
        Bit#(512) d <- dram.read;
        serial_dramQ.put(d);
    endrule

    /* Write back to DMA */
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

    /* Giving DMA write done signal to the HOST */
    rule sendResultToHost; 
        let r <- pcie.dataReq;
        let a = r.addr;
        let offset = (a>>2);
        if ( offset == 0 ) begin
            dramWriteDoneSignalQ.deq;
            pcie.dataSend(r, 1);
        end else begin
            dmaWriteDoneSignalQ.deq;
            pcie.dataSend(r, 1);
        end
    endrule
endmodule
