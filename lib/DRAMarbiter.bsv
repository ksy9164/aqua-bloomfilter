package DRAMarbiter;

import FIFO::*;
import Vector::*;
import DRAMController::*;
import Serializer::*;
import FIFOLI::*;

import BRAM::*;
import BRAMFIFO::*;

interface GetIfc;
    method ActionValue#(Bit#(512)) get;
endinterface

interface PutIfc;
    method Action put(Bit#(512) d);
endinterface

interface DRAMarbiterlIfc#(numeric type rw_chunk_size, numeric type user_num);
    method Action req(Tuple4#(Bit#(32), Bit#(1), Bit#(16), Bit#(3)) d); // addr, read/write, 
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

    FIFO#(Tuple4#(Bit#(32), Bit#(1), Bit#(16), Bit#(3))) inQ <- mkSizedBRAMFIFO(16);
//ex) inQ.enq(tuple4(adress, read/write, how many chunks req, id)

Reg#(Bit#(2)) dram_arbiter_handle <- mkReg(0);

Reg#(Bit#(32)) target_read_addr <- mkReg(0);
Reg#(Bit#(32)) target_write_addr <- mkReg(0);

Reg#(Bit#(16)) target_read_cnt <- mkReg(0);
Reg#(Bit#(16)) target_write_cnt <- mkReg(0);
Reg#(Bit#(3)) target_id <- mkReg(0);
Reg#(Bit#(16)) dram_read_cnt <- mkReg(0);
Reg#(Bit#(16)) dram_read_req_cnt <- mkReg(0);
Reg#(Bit#(16)) dram_write_cnt <- mkReg(0);
Reg#(Bit#(16)) dram_write_t_cnt <- mkReg(0);

Reg#(Bit#(64)) clock_cnt <- mkReg(0);

/* Vector#(user_num, FIFO#(Tuple2#(Bit#(512), Bit#(3)))) temp_outQ <- replicateM(mkFIFO); */
Vector#(user_num, FIFO#(Tuple2#(Bit#(512), Bit#(3)))) temp_outQ <- replicateM(mkSizedBRAMFIFO(1024));
Vector#(user_num, FIFO#(Bit#(512))) outQ <- replicateM(mkSizedBRAMFIFO(1024));

Vector#(user_num, SerializerIfc#(512, 4)) serial_outQ <- replicateM(mkSerializer);
Vector#(user_num, DeSerializerIfc#(128, 4)) deserial_inQ <- replicateM(mkDeSerializer);

/* Vector#(user_num, FIFO#(Bit#(512))) outQ <- replicateM(mkFIFO); */

/* Vector#(user_num, FIFO#(Bit#(512))) temp_inQ <- replicateM(mkFIFO); */
Vector#(user_num, FIFO#(Bit#(512))) t_writeQ <- replicateM(mkFIFO);
/* FIFOLI#(Bit#(512), 3) writeQ <- mkFIFOLI; */
FIFO#(Bit#(512)) writeQ <- mkSizedBRAMFIFO(1024);
rule clock_go;
    clock_cnt <= clock_cnt + 1;
endrule

rule get_requset_from_user(dram_arbiter_handle == 0);
    inQ.deq;
    let d = inQ.first;
    Bit#(16) t_num = tpl_3(d);

    target_read_addr <= tpl_1(d);
    target_write_addr <= tpl_1(d);
    target_read_cnt <= fromInteger(valueOf(target_size)) * t_num;
    target_write_cnt <= fromInteger(valueOf(target_size)) * t_num;
    target_id <= tpl_4(d);

    Bit#(2) handle = zeroExtend(tpl_2(d));
    if (handle == 0) begin 
        dram_arbiter_handle <= 1; // read req
    end else begin
        dram_arbiter_handle <= 3; // write req
    end
endrule

FIFO#(Bit#(3)) target_read_idQ <- mkSizedBRAMFIFO(64);
rule readReqStart(dram_arbiter_handle == 1 && dram_read_req_cnt != target_read_cnt);
    Bit#(32) cnt = zeroExtend(dram_read_req_cnt) * 64;
    Bit#(32) t_addr = target_read_addr + cnt;
    dram.readReq(zeroExtend(t_addr), 64);

    if (dram_read_req_cnt + 1 == target_read_cnt) begin
        dram_read_req_cnt <= 0;
        dram_arbiter_handle <= 0;
    end else begin
        dram_read_req_cnt <= dram_read_req_cnt + 1;
    end
    target_read_idQ.enq(target_id);
endrule

rule getReadData;
        Bit#(512) d <- dram.read;
        target_read_idQ.deq;
        let t_id = target_read_idQ.first;
        temp_outQ[0].enq(tuple2(d, t_id));
endrule

for (Bit#(16) i = 0; i < fromInteger(valueOf(user_num)); i = i + 1) begin
    rule read_output_relay;
        temp_outQ[i].deq;
        let d = temp_outQ[i].first;
        Bit#(16) t_id = zeroExtend(tpl_2(d));
        if (t_id == i) begin
            outQ[i].enq(tpl_1(d));
        end else if( i < fromInteger(valueOf(user_num) - 1)) begin
            temp_outQ[i + 1].enq(d);
        end
    endrule
end

/* for (Integer i = 0; i < fromInteger(valueOf(user_num)); i = i + 1) begin
 *     rule read_input_relay(fromInteger(i) == pack(target_id) && dram_arbiter_handle == 3);
 *         let d <- deserial_inQ[i].get;
 *         t_writeQ[i].enq(d);
 *     endrule
 * end */

for (Integer i = 0; i < fromInteger(valueOf(user_num)) ; i = i + 1) begin
    rule read_input_relay(fromInteger(i) == pack(target_id) && dram_arbiter_handle == 3 && dram_write_t_cnt != target_write_cnt);
        dram_write_t_cnt <= dram_write_t_cnt + 1;
        t_writeQ[i].deq;
        let d = t_writeQ[i].first;
        writeQ.enq(d);
    endrule    
end

rule write_dram_data(dram_write_cnt != target_write_cnt && dram_arbiter_handle == 3);
    writeQ.deq;
    Bit#(512) d = writeQ.first;
    Bit#(32) cnt = zeroExtend(dram_write_cnt) * 64;

    Bit#(32) idx = target_write_addr + cnt;
    dram.write(zeroExtend(idx), d , 64);

    if (dram_write_cnt == target_write_cnt - 1 && dram_write_t_cnt == target_write_cnt) begin
        dram_arbiter_handle <= 0;
        dram_write_cnt <= 0;
        dram_write_t_cnt <= 0;
    end else begin
        dram_write_cnt <= dram_write_cnt + 1;
    end
endrule

Vector#(user_num, PutIfc) put_;
Vector#(user_num, GetIfc) get_;

    for (Integer i = 0; i < valueOf(user_num); i = i + 1) begin
        get_[i] = interface GetIfc;
            method ActionValue#(Bit#(512)) get;
                outQ[i].deq;
                return outQ[i].first;
                /* let d <- serial_outQ[i].get;
                 * return d; */
            endmethod
        endinterface;
    end

    for (Integer i = 0; i < valueOf(user_num); i = i + 1) begin
        put_[i] = interface PutIfc;
            method Action put(Bit#(512) d);
                t_writeQ[i].enq(d);
            endmethod
        endinterface;
    end

    interface get = get_;
    interface put = put_;
method Action req(Tuple4#(Bit#(32), Bit#(1), Bit#(16), Bit#(3)) d);
    inQ.enq(d);
endmethod

endmodule
endpackage:DRAMarbiter
