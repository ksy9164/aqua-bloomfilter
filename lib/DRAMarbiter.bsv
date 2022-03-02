package DRAMarbiter;

import FIFO::*;
import Vector::*;
import DRAMController::*;


interface GetIfc;
    method ActionValue#(Bit#(512)) get;
endinterface

interface PutIfc;
    method Action put(Bit#(512) d);
endinterface

interface DRAMarbiterlIfc#(numeric type rw_chunk_size, numeric type user_num);
    method Action req(Tuple4#(Bit#(32), Bit#(1), Bit#(4), Bit#(3)) d); // addr, read/write, 
    interface Vector#(user_num, GetIfc) get;
    interface Vector#(user_num, PutIfc) put;
endinterface
   
typedef 4   DivSize;
typedef 4   Module_num;
// ex) DRAMarbiterlIfc#(Bit#(32), 2048, Bit#(3) , 8)
// Bit#(32) address data for accessing DRAM
// 2048 Byte per one read / write request
// Bit#(3) for addressing user_id
// 8 users who are using Arbiter

module mkDRAMarbiter#(DRAMUserIfc dram)
    (DRAMarbiterlIfc#(rw_chunk_size, user_num))
    provisos(
        Div#(rw_chunk_size, 64, target_size),
        Add#(user_num, 1, a__)
    );

FIFO#(Tuple4#(Bit#(32), Bit#(1), Bit#(4), Bit#(3))) inQ <- mkFIFO;
//ex) inQ.enq(tuple4(adress, read/write, how many chunks req, id)

Reg#(Bit#(3)) dram_arbiter_handle <- mkReg(0);
FIFO#(Bit#(512)) merged_inpiutQ <- mkFIFO;

Reg#(Bit#(32)) target_addr <- mkReg(0);
Reg#(Bit#(32)) target_cnt <- mkReg(0);
Reg#(Bit#(3)) target_id <- mkReg(0);
Reg#(Bit#(32)) dram_read_cnt <- mkReg(0);
Reg#(Bit#(32)) dram_read_req_cnt <- mkReg(0);
Reg#(Bit#(32)) dram_write_cnt <- mkReg(0);

Vector#(user_num, FIFO#(Tuple2#(Bit#(512), Bit#(3)))) temp_outQ <- replicateM(mkFIFO);
Vector#(user_num, FIFO#(Bit#(512))) outQ <- replicateM(mkFIFO);

Vector#(user_num, FIFO#(Bit#(512))) temp_inQ <- replicateM(mkFIFO);
Vector#(user_num, FIFO#(Bit#(512))) t_writeQ <- replicateM(mkFIFO);
FIFO#(Bit#(512)) writeQ <- mkFIFO;

rule get_requset_from_user(dram_arbiter_handle == 0);
    inQ.deq;
    let d = inQ.first;
    target_addr <= tpl_1(d);

    Bit#(2) handle = zeroExtend(tpl_2(d));
    if (handle == 0) begin 
        dram_arbiter_handle <= 1; // read req
    end else begin
        dram_arbiter_handle <= 3; // write req
    end
    
    Bit#(32) t_num = zeroExtend(tpl_3(d));
    target_cnt <= fromInteger(valueOf(target_size)) * t_num;

    target_id <= tpl_4(d);
endrule

rule readReqStart(dram_arbiter_handle == 1 && dram_read_req_cnt != target_cnt);
    Bit#(32) t_addr = target_addr + (dram_read_req_cnt * 64);
    dram.readReq(zeroExtend(t_addr), 64);
    dram_read_req_cnt <= dram_read_req_cnt + 1;
endrule

rule getReadData(dram_arbiter_handle == 1 && dram_read_cnt != target_cnt);
        if (dram_read_cnt == target_cnt) begin
            dram_arbiter_handle <= 2;
        end
        Bit#(512) d <- dram.read;
        dram_read_cnt <= dram_read_cnt + 1;
        temp_outQ[0].enq(tuple2(d, target_id));
endrule

rule resetRun(dram_arbiter_handle == 3);
    target_addr <= 0;
    target_cnt <= 0;
    target_id <= unpack(0);
    dram_arbiter_handle <= 4;
endrule

rule resetTempValue(dram_arbiter_handle == 4);
    dram_arbiter_handle <= 0;
    dram_read_cnt <= 0;
    dram_read_req_cnt <= 0;
    dram_write_cnt <= 0;
endrule

for (Bit#(32) i = 0; i < fromInteger(valueOf(user_num)); i = i + 1) begin
    rule read_output_relay;
        temp_outQ[i].deq;
        let d = temp_outQ[i].first;
        Bit#(32) t_id = zeroExtend(tpl_2(d));
        if (t_id == i) begin
            outQ[i].enq(tpl_1(d));
        end else if( i < fromInteger(valueOf(user_num) - 1)) begin
            temp_outQ[i + 1].enq(d);
        end
    endrule
end

for (Integer i = 0; i < fromInteger(valueOf(user_num)); i = i + 1) begin
    rule read_input_relay(fromInteger(i) == pack(target_id) && dram_arbiter_handle == 3);
        temp_inQ[i].deq;
        let d = temp_inQ[i].first;
        t_writeQ[i].enq(d);
    endrule    
end

for (Integer i = 0; i < fromInteger(valueOf(user_num)) ; i = i + 1) begin
    rule read_input_relay(fromInteger(i) == pack(target_id) && dram_arbiter_handle == 3);
        t_writeQ[i].deq;
        let d = t_writeQ[i].first;
        if (i < fromInteger(valueOf(user_num)) - 1) begin
            t_writeQ[i + 1].enq(d);
        end else begin
            writeQ.enq(d);
        end
    endrule    
end

rule write_dram_data(dram_write_cnt != target_cnt && dram_arbiter_handle == 3);
    writeQ.deq;
    Bit#(512) d = writeQ.first;
    Bit#(32) idx = target_addr + (target_cnt * 64);
    dram.write(zeroExtend(idx), d , 64);
    dram_write_cnt <= dram_write_cnt + 1;

    if (dram_write_cnt == target_cnt - 1) begin
        dram_arbiter_handle <= 2;
    end
endrule

Vector#(user_num, PutIfc) put_;
Vector#(user_num, GetIfc) get_;

    for (Integer i = 0; i < valueOf(user_num); i = i + 1) begin
        get_[i] = interface GetIfc;
            method ActionValue#(Bit#(512)) get;
                outQ[i].deq;
                return outQ[i].first;
            endmethod
        endinterface;
    end

    for (Integer i = 0; i < valueOf(user_num); i = i + 1) begin
        put_[i] = interface PutIfc;
            method Action put(Bit#(512) d);
                temp_inQ[i].enq(d);
            endmethod
        endinterface;
    end

    interface get = get_;
    interface put = put_;
method Action req(Tuple4#(Bit#(32), Bit#(1), Bit#(4), Bit#(3)) d);
    inQ.enq(d);
endmethod

endmodule
endpackage:DRAMarbiter