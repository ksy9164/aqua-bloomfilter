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

    Vector#(2, FIFOLI#(Bit#(64), 3)) spread_inputQ <- replicateM(mkFIFOLI);

    Vector#(4, FIFO#(Bit#(64))) merge_st1Q <- replicateM(mkFIFO);
    Vector#(2, FIFO#(Bit#(128))) merge_st2Q <- replicateM(mkFIFO);

    Vector#(4, BloomNodeIfc) node <- replicateM(mkBloomNode);

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
/*
	BLRadixIfc#(10,3,4,Bit#(32),0,7) radixSub <- mkBLRadix;
	Reg#(Bit#(32)) dataInputCounter <- mkReg(0);
	FIFO#(Vector#(4, Bit#(32))) toUploader <- mkFIFO;

	Integer dataCnt = 1024*512;

	rule inputData(dataInputCounter<fromInteger(dataCnt));
        Bit#(128) t <- serial_to_radixQ.get;
		dataInputCounter <= dataInputCounter + 1;
		Vector#(4,Bit#(32)) ind;
		
		for (Bit#(4) i = 0; i < 4; i = i + 1) begin
		    ind[i] = (ind[i]) ^ (ind[3 - i]) * dataInputCounter;
		end
		ind[0] = t[127:96];
		ind[1] = t[95:64];
		ind[2] = t[63:32];
		ind[3] = t[31:0];
        $display("Before sorting %d %d %d %d ", ind[0], ind[1], ind[2], ind[3]);

		radixSub.enq(ind);

		if ( dataInputCounter + 1 >= fromInteger(dataCnt) ) begin
		    $display("Burst!! ");
			radixSub.flush();
		end
	endrule

	Reg#(Bit#(32)) burstTotal <- mkReg(0);
	Reg#(Bit#(32)) startCycle <- mkReg(0);
	rule flushBurstReady;
		let d <- radixSub.burstReady;
		burstTotal <= burstTotal + zeroExtend(d);
	endrule

	Reg#(Bit#(32)) dataOutputCounter <- mkReg(0);
	rule readOutput;
		Vector#(4,Bit#(32)) outd = radixSub.first;
		radixSub.deq;
		toUploader.enq(outd);
		dataOutputCounter <= dataOutputCounter + 1;
	endrule


    Vector#(4, FIFO#(Bit#(32))) toNodeQ <- replicateM(mkFIFO);
    Reg#(Bit#(32)) run_cnt <- mkReg(0); 
    Reg#(Bit#(32)) clock_cnt <- mkReg(0); 

    rule get_Data_from_radix_sorter;
        toUploader.deq;
        Vector#(4, Bit#(32)) t = toUploader.first;
        toNodeQ[0].enq(t[0]);
        toNodeQ[1].enq(t[1]);
        toNodeQ[2].enq(t[2]);
        toNodeQ[3].enq(t[3]);
        $display("After sorting %d %d %d %d ", t[0], t[1], t[2], t[3]);
        run_cnt <= run_cnt + 1;
        $display("runcnt is %d ", run_cnt);
    endrule
*/

    /// for arbiter testing
    Vector#(4, FIFO#(Bit#(32))) toNodeQ <- replicateM(mkFIFO);
    Reg#(Bit#(32)) run_cnt <- mkReg(0); 
    Reg#(Bit#(32)) clock_cnt <- mkReg(0); 
    rule get_Data_from_radix_sorter;
        Bit#(128) t <- serial_to_radixQ.get;
        toNodeQ[0].enq(t[127:96]);
        toNodeQ[1].enq(t[95:64]);
        toNodeQ[2].enq(t[63:32]);
        toNodeQ[3].enq(t[31:0]);
        //run_cnt <= run_cnt + 1;
        $display("runcnt is %d ", run_cnt);
    endrule
/////////////////////////// DRAM Arbiter part ////////////

    Reg#(Bit#(2)) rullet <- mkReg(0);
    Reg#(Bit#(32)) req_counter <- mkReg(0);

    Reg#(Bit#(1)) req_merger <- mkReg(0);
    DRAMarbiterlIfc#(2048, 4) dramArbiter <- mkDRAMarbiter(dram);

    for (Bit#(4) i = 0; i < 4; i = i + 1) begin
        rule put_data_to_node;
            toNodeQ[i].deq;
            node[i].enq(toNodeQ[i].first);
        endrule
    end

    Vector#(2, FIFO#(Tuple4#(Bit#(32), Bit#(1), Bit#(4), Bit#(3)))) dram_reqQ <- replicateM(mkFIFO);

    rule rullet_rule;
        rullet <= rullet + 1;
        //clock_cnt <= clock_cnt + 1;
    endrule

    rule get_req;
        let write_addr <- node[rullet].dram_write_req;
        let read_addr <- node[rullet].dram_read_req;
        //req_counter <= req_counter + 1;
        $display("clock %d , run %d ", clock_cnt, run_cnt);
        $display("req counting is %d ", req_counter);

        dram_reqQ[0].enq(tuple4(zeroExtend(write_addr), 1, 4, zeroExtend(rullet)));
        dram_reqQ[1].enq(tuple4(zeroExtend(read_addr), 0, 4, zeroExtend(rullet)));
    endrule

    rule put_req;
        dram_reqQ[req_merger].deq;
        dramArbiter.req(dram_reqQ[req_merger].first);
        req_merger <= req_merger + 1;
    endrule

    for (Bit#(4) i = 0; i < 4; i = i + 1) begin
        rule putDRAMdata;
            let d <- node[i].dram_write_data_get;
            dramArbiter.put[i].put(d);
        endrule

        rule getDRAMdata;
            let d <- dramArbiter.get[i].get;
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
