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
    DeSerializerIfc#(8, 64) deserial_dram <- mkDeSerializer;
    SerializerIfc#(512, 4) serial_dramQ <- mkSerializer;

    FIFO#(Bit#(32)) dmaReadReqQ <- mkFIFO;
    FIFO#(Bit#(32)) dmaWriteReqQ <- mkFIFO;
    FIFO#(Bit#(1)) dramWriteDoneSignalQ <- mkFIFO;
    FIFO#(Bit#(1)) dmaWriteDoneSignalQ <- mkFIFO;

    Reg#(Bit#(32)) readCnt <- mkReg(0);

    Reg#(Bit#(1)) dmaWriteHandle <- mkReg(0);
    Reg#(Bit#(32)) dmaWriteTarget <- mkReg(0);
    Reg#(Bit#(32)) dmaWriteCnt <- mkReg(0);

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

    rule mergingForDRAM;
        Bit#(8) d <- serial_input.get;
        deserial_dram.put(d);
    endrule

    /* Write to DRAM */
    rule dramWrite(dramWriteCnt < file_size);
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
