#include <systemc.h>
#include <connections/connections.h>
#include <ac_int.h>

#ifndef __JBFIFO_H__
#define __JBFIFO_H__

template <int W, int D>
SC_MODULE( JbFifo){

    sc_in<bool> SC_NAMED(rstn);
    sc_in<bool> SC_NAMED(clk);
    Connections::In<ac_int<W,false>> SC_NAMED(ch_din );
    Connections::Out<ac_int<W,false>>SC_NAMED(ch_dout);
    sc_out<bool> SC_NAMED(o_empty);
    sc_out<bool> SC_NAMED(o_full );

    #pragma hls_aggregate variable=mem
    ac_int<W,false> mem[2][D/2];
    ac_int<W,false> din, reg_din, rdat;
    ac_int<ac::log2_ceil<D>::val,false> wptr, rptr;
    ac_int<1,false> wptr_bank, rptr_bank;
    ac_int<ac::log2_ceil<D/2>::val,false> wptr_h, rptr_h;
    ac_int<ac::nbits<D>::val,false> dcnt;

    SC_HAS_PROCESS(JbFifo);
    JbFifo(sc_module_name name)
        : sc_module(name)
    {
        SC_THREAD(run);
        sensitive << clk.pos();
        async_reset_signal_is(rstn, false);
    }

    void run(){
        ch_din.Reset();
        ch_dout.Reset();
	o_empty = 0; o_full = 0;
	bool is_pop = false;
        ac_int<1,false> mem_read = 0;
        ac_int<ac::nbits<D>::val,false> rptr_tmp=0, wptr_tmp=0;
        bool out_pend = false, rdat_rdy =false;
        bool has_room = true, has_data = false;
        bool wmem_rdy = false, wptr_incr = false;
        wptr = 0; wptr_bank = 0; wptr_h = 0; 
        rptr = 0; rptr_bank = 0; rptr_h = 0;
        dcnt = 0;
        din  = 0; reg_din = 0;
        wait();
        while(1){
            wptr_bank = wptr[0];
            rptr_bank = rptr[0];
            wptr_h = wptr >>1;
            rptr_h = rptr >>1;
            //if( has_room ){
            //    is_pop = ch_din.PopNB(din);
            //}
            if( !is_pop ){
                is_pop = ch_din.PopNB(din);
            }
            if( is_pop && has_room){
                reg_din = din;
                wmem_rdy = true;
		is_pop = false;
            }

            if(rdat_rdy){
                out_pend = !ch_dout.PushNB(rdat);
                rdat_rdy = out_pend;
            }

            mem_read = has_data && !out_pend &&
                       (wmem_rdy && (wptr_bank != rptr_bank) || !wmem_rdy);
            if(mem_read){
                rdat = mem[rptr_bank][rptr_h]; 
                rdat_rdy = true;
            }
            rptr_tmp =  rptr + mem_read;
            rptr = (rptr_tmp >= D) ? ac_int<ac::nbits<D>::val, false>(0)
                                   : rptr_tmp;

            wptr_incr = wmem_rdy;
            if(wmem_rdy){
                mem[wptr_bank][wptr_h] = reg_din;
                wmem_rdy = false;
            }
            assert(!(mem_read && wptr_incr && wptr_bank == rptr_bank));
            dcnt = dcnt + wptr_incr - mem_read;
            wptr_tmp =  wptr + wptr_incr;
            wptr = (wptr_tmp >= D) ? ac_int<ac::nbits<D>::val, false>(0)
                                   : wptr_tmp;

            has_room = (dcnt < D);
            has_data = (dcnt > 0);
	    o_empty  = (dcnt == 0);
	    o_full   = (dcnt >= D);

            wait();
        }

    }
};

#endif
