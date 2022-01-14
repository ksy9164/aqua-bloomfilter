/* This module scatters same data to each output FIFO */

package MultiN;
import FIFO::*;
import Vector::*;
import FIFOLI::*;

interface GetIfc#(type t);
    method ActionValue#(t) get;
endinterface

interface MultiOneToFourIfc#(type t);
    method Action enq(t d);
    interface Vector#(4, GetIfc#(t)) get;
endinterface

module mkMultiOnetoFour (MultiOneToFourIfc#(t))
    provisos (
        Bits#(t , a__)
    );
    FIFO#(t) inQ <- mkFIFO;
    Vector#(4, FIFOLI#(t, 2)) outQ <- replicateM(mkFIFOLI);
    Vector#(2, FIFO#(t)) tempQ <- replicateM(mkFIFO);

    rule ontToTwo;
        inQ.deq;
        t d = inQ.first;
        tempQ[0].enq(d);
        tempQ[1].enq(d);
    endrule

    for (Bit#(4) i = 0; i < 2; i = i + 1) begin
        rule twoToFour;
            tempQ[i].deq;
            t d = tempQ[i].first;
            outQ[i * 2].enq(d);
            outQ[i * 2 + 1].enq(d);
        endrule
    end

    Vector#(4, GetIfc#(t)) get_;
    for (Integer i = 0; i < 4; i = i+1) begin
        get_[i] = interface GetIfc;
            method ActionValue#(t) get;
                outQ[i].deq;
                return outQ[i].first;
            endmethod
        endinterface;
    end
    interface get = get_;

    method Action enq(t d);
        inQ.enq(d);
    endmethod

endmodule

interface MultiOneToEightIfc#(type t);
    method Action enq(t d);
    interface Vector#(8, GetIfc#(t)) get;
endinterface

module mkMultiOnetoEight (MultiOneToEightIfc#(t))
    provisos (
        Bits#(t , a__)
    );
    FIFO#(t) inQ <- mkFIFO;
    Vector#(8, FIFO#(t)) outQ <- replicateM(mkFIFO);
    Vector#(2, FIFO#(t)) temp_1Q <- replicateM(mkFIFO);
    Vector#(4, FIFO#(t)) temp_2Q <- replicateM(mkFIFO);

    rule ontToTwo;
        inQ.deq;
        t d = inQ.first;
        temp_1Q[0].enq(d);
        temp_1Q[1].enq(d);
    endrule

    for (Bit#(4) i = 0; i < 2; i = i + 1) begin
        rule twoToFour;
            temp_1Q[i].deq;
            t d = temp_1Q[i].first;
            temp_2Q[i * 2].enq(d);
            temp_2Q[i * 2 + 1].enq(d);
        endrule
    end

    for (Bit#(4) i = 0; i < 4; i = i + 1) begin
        rule fourToEight;
            temp_2Q[i].deq;
            t d = temp_2Q[i].first;
            outQ[i * 2].enq(d);
            outQ[i * 2 + 1].enq(d);
        endrule
    end

    Vector#(8, GetIfc#(t)) get_;
    for (Integer i = 0; i < 8; i = i+1) begin
        get_[i] = interface GetIfc;
            method ActionValue#(t) get;
                outQ[i].deq;
                return outQ[i].first;
            endmethod
        endinterface;
    end
    interface get = get_;

    method Action enq(t d);
        inQ.enq(d);
    endmethod

endmodule
endpackage: MultiN
