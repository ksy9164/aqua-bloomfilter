/* This modules hashs 64bits to 64bits for bloomfilter */

package BloomHash;
import FIFO::*;
import Vector::*;

interface BloomHashIfc;
    method Action enq(Bit#(64) d);
    method ActionValue#(Bit#(32)) get;
endinterface

function Bit#(32) jshash (Bit#(32) hash, Bit#(8) d);
    return hash ^ ((hash<<5) + zeroExtend(d) + (hash>>2));
endfunction

function Bit#(32) sdbmhash (Bit#(32) hash, Bit#(8) d);
    Bit#(32) data = zeroExtend(d);
    return data + (hash << 6) + (hash << 16) - hash;
endfunction

function Bit#(32) djbhash (Bit#(32) hash, Bit#(8) d);
    Bit#(32) data = zeroExtend(d);
    return ((hash << 5) + hash ) + data;
endfunction

function Bit#(32) addictivehash (Bit#(32) hash, Bit#(8) d);
    Bit#(32) data = zeroExtend(d);
    return hash + data;
endfunction

function Bit#(32) bernhash (Bit#(32) hash, Bit#(8) d);
    Bit#(32) data = zeroExtend(d);
    return hash * 33 + data;
endfunction

function Bit#(32) bkdrhash (Bit#(32) hash, Bit#(8) d);
    Bit#(32) seed = 131;
    Bit#(32) data = zeroExtend(d);
    return hash * seed + data;
endfunction

function Bit#(32) dekhash (Bit#(32) hash, Bit#(8) d);
    Bit#(32) data = zeroExtend(d);
    return ((hash << 5) ^ (hash >> 27)) ^ (data);
endfunction

function Bit#(32) rotatinghash (Bit#(32) hash, Bit#(8) d);
    Bit#(32) data = zeroExtend(d);
    return (hash << 4) ^ (hash >> 28) ^ data;
endfunction

module mkJsHash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(32)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(32))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(32) hash_init = 1315423911;

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
    method ActionValue#(Bit#(32)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule

module mkSdbmHash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(32)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(32))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(32) hash_init = 0;

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
    method ActionValue#(Bit#(32)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule

module mkDjbHash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(32)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(32))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(32) hash_init = 5381;

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
    method ActionValue#(Bit#(32)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule

module mkAddictiveHash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(32)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(32))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(32) hash_init = 0;

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
                hashQ[i].enq(addictivehash(hashQ[i - 1].first, input8bitsQ[i].first));
            end else begin
                input8bitsQ[i].deq;
                hashQ[i].enq(addictivehash(hash_init, input8bitsQ[i].first));
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
    method ActionValue#(Bit#(32)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule

module mkBernsteinHash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(32)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(32))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(32) hash_init = 0;

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
                hashQ[i].enq(bernhash(hashQ[i - 1].first, input8bitsQ[i].first));
            end else begin
                input8bitsQ[i].deq;
                hashQ[i].enq(bernhash(hash_init, input8bitsQ[i].first));
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
    method ActionValue#(Bit#(32)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule

module mkBkdrHash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(32)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(32))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(32) hash_init = 0;

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
                hashQ[i].enq(bkdrhash(hashQ[i - 1].first, input8bitsQ[i].first));
            end else begin
                input8bitsQ[i].deq;
                hashQ[i].enq(bkdrhash(hash_init, input8bitsQ[i].first));
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
    method ActionValue#(Bit#(32)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule

module mkDekHash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(32)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(32))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(32) hash_init = 8;

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
                hashQ[i].enq(dekhash(hashQ[i - 1].first, input8bitsQ[i].first));
            end else begin
                input8bitsQ[i].deq;
                hashQ[i].enq(dekhash(hash_init, input8bitsQ[i].first));
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
    method ActionValue#(Bit#(32)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule

module mkRotatingHash (BloomHashIfc);
    Vector#(2, FIFO#(Bit#(32))) inQ <- replicateM(mkFIFO);
    FIFO#(Bit#(32)) outQ <- mkFIFO;
    Vector#(8, FIFO#(Bit#(32))) hashQ <- replicateM(mkFIFO);
    Vector#(8, FIFO#(Bit#(8))) input8bitsQ <- replicateM(mkFIFO);
    Bit#(32) hash_init = 8;

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
                hashQ[i].enq(rotatinghash(hashQ[i - 1].first, input8bitsQ[i].first));
            end else begin
                input8bitsQ[i].deq;
                hashQ[i].enq(rotatinghash(hash_init, input8bitsQ[i].first));
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
    method ActionValue#(Bit#(32)) get;
        outQ.deq;
        return outQ.first;
    endmethod
endmodule

endpackage
