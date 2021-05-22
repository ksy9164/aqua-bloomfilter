/* This modules hashs 64bits to 64bits for bloomfilter */

package BloomHash;
import FIFO::*;
import Vector::*;

interface BloomHashIfc;
    method Action enq(Bit#(64) d);
    method ActionValue#(Bit#(64)) get;
endinterface

function Bit#(64) jshash (Bit#(64) hash, Bit#(8) d);
    return hash ^ ((hash<<5) + zeroExtend(d) + (hash>>2));
endfunction

function Bit#(64) elfhash (Bit#(64) hash, Bit#(8) d);
    Bit#(64) x = 0;
    Bit#(64) rand_val = 64'hf000000000000000;

    hash = (hash << 4) + zeroExtend(d);
    x = hash & rand_val;

    if (x != 0) begin
        hash = hash ^ (x >> 24);
    end
    return hash & ~x;
endfunction

function Bit#(64) sdbmhash (Bit#(64) hash, Bit#(8) d);
    Bit#(64) data = zeroExtend(d);
    return data + (hash << 6) + (hash << 16) - hash;
endfunction

function Bit#(64) djbhash (Bit#(64) hash, Bit#(8) d);
    Bit#(64) data = zeroExtend(d);
    return ((hash << 5) + hash ) + data;
endfunction

module mkJshash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(64)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(64))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(64) hash_init = 1315423911;

    for (Bit#(8) i = 0 ; i < 2; i = i +1) begin
        rule inputTo4ways;
            inQ[i].deq;
            Bit#(32) d = inQ[i].first;
            for (Bit#(8) j = 0; j < 4; j = j + 1) begin
                input8bitsQ[i * 4 + j].enq(truncate(d));
                d = d >> 8;
            end
        endrule
    end

    for (Bit#(8) i = 0; i < 8; i = i +1) begin
        rule hashing;
            if (i != 0) begin
                hashQ[i - 1].deq;
                input8bitsQ[i].deq;
                hashQ[i].enq(jshash(hashQ[i - 1].first, input8bitsQ[i].first));
            end else begin
                input8bitsQ[i].deq;
                hashQ[i].enq(jshash(hash_init, input8bitsQ[i].first));
            end
        endrule
    end

    rule makeOutput;
        hashQ[7].deq;
        outQ.enq(hashQ[7].first);
    endrule
    method Action enq(Bit#(64) d);
        Bit#(32) upper = truncate(d >> 32);
        Bit#(32) lower = truncate(d);
        inQ[0].enq(lower);
        inQ[1].enq(upper);
    endmethod
    method ActionValue#(Bit#(64)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule

module mkElfhash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(64)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(64))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(64) hash_init = 0;

    for (Bit#(8) i = 0 ; i < 2; i = i +1) begin
        rule inputTo4ways;
            inQ[i].deq;
            Bit#(32) d = inQ[i].first;
            for (Bit#(8) j = 0; j < 4; j = j + 1) begin
                input8bitsQ[i * 4 + j].enq(truncate(d));
                d = d >> 8;
            end
        endrule
    end

    for (Bit#(8) i = 0; i < 8; i = i +1) begin
        rule hashing;
            if (i != 0) begin
                hashQ[i - 1].deq;
                input8bitsQ[i].deq;
                hashQ[i].enq(elfhash(hashQ[i - 1].first, input8bitsQ[i].first));
            end else begin
                input8bitsQ[i].deq;
                hashQ[i].enq(elfhash(hash_init, input8bitsQ[i].first));
            end
        endrule
    end

    rule makeOutput;
        hashQ[7].deq;
        outQ.enq(hashQ[7].first);
    endrule
    method Action enq(Bit#(64) d);
        Bit#(32) upper = truncate(d >> 32);
        Bit#(32) lower = truncate(d);
        inQ[0].enq(lower);
        inQ[1].enq(upper);
    endmethod
    method ActionValue#(Bit#(64)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule

module mkSdbmhash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(64)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(64))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(64) hash_init = 0;

    for (Bit#(8) i = 0 ; i < 2; i = i +1) begin
        rule inputTo4ways;
            inQ[i].deq;
            Bit#(32) d = inQ[i].first;
            for (Bit#(8) j = 0; j < 4; j = j + 1) begin
                input8bitsQ[i * 4 + j].enq(truncate(d));
                d = d >> 8;
            end
        endrule
    end

    for (Bit#(8) i = 0; i < 8; i = i +1) begin
        rule hashing;
            if (i != 0) begin
                hashQ[i - 1].deq;
                input8bitsQ[i].deq;
                hashQ[i].enq(sdbmhash(hashQ[i - 1].first, input8bitsQ[i].first));
            end else begin
                input8bitsQ[i].deq;
                hashQ[i].enq(sdbmhash(hash_init, input8bitsQ[i].first));
            end
        endrule
    end

    rule makeOutput;
        hashQ[7].deq;
        outQ.enq(hashQ[7].first);
    endrule
    method Action enq(Bit#(64) d);
        Bit#(32) upper = truncate(d >> 32);
        Bit#(32) lower = truncate(d);
        inQ[0].enq(lower);
        inQ[1].enq(upper);
    endmethod
    method ActionValue#(Bit#(64)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule

module mkDjbhash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(64)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(64))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(64) hash_init = 5381;

    for (Bit#(8) i = 0 ; i < 2; i = i +1) begin
        rule inputTo4ways;
            inQ[i].deq;
            Bit#(32) d = inQ[i].first;
            for (Bit#(8) j = 0; j < 4; j = j + 1) begin
                input8bitsQ[i * 4 + j].enq(truncate(d));
                d = d >> 8;
            end
        endrule
    end

    for (Bit#(8) i = 0; i < 8; i = i +1) begin
        rule hashing;
            if (i != 0) begin
                hashQ[i - 1].deq;
                input8bitsQ[i].deq;
                hashQ[i].enq(djbhash(hashQ[i - 1].first, input8bitsQ[i].first));
            end else begin
                input8bitsQ[i].deq;
                hashQ[i].enq(djbhash(hash_init, input8bitsQ[i].first));
            end
        endrule
    end

    rule makeOutput;
        hashQ[7].deq;
        outQ.enq(hashQ[7].first);
    endrule
    method Action enq(Bit#(64) d);
        Bit#(32) upper = truncate(d >> 32);
        Bit#(32) lower = truncate(d);
        inQ[0].enq(lower);
        inQ[1].enq(upper);
    endmethod
    method ActionValue#(Bit#(64)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule
endpackage
