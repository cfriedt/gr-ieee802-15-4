#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2015 <+YOU OR YOUR COMPANY+>.
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

import time

from itertools import repeat

from gnuradio import gr, gr_unittest
from gnuradio import blocks
import ieee802154_swig as ieee802154

import pmt

class qa_packet_sink (gr_unittest.TestCase):

    # Synchronization Header (SHR) 
    ## Preamble (4 bytes)
    preamble = list( repeat( 0, 32 ) )
    ## Start of Frame Descriptor (SFD) (1 byte)
    sfd = [ 1, 1, 1, 0, 0, 1, 0, 1 ]
    # PHY Header (PHR) (1 byte)
    ## Frame Length
    frame_length = [ 1, 0, 1, 0, 0, 0, 0, 0 ]
    # MAC Header (MHR)
    ## Frame Control (2 bytes)
    fc = [ 
    ### Frame Type (Acknowledgement)
        0, 1, 0,
    ### Security Enabled
        0,
    ### Frame Pending
        0,
    ### Acknowledgement Request
        0,
    ### PAN ID Compression
        0,
    ### Reserved
        0, 0, 0,
    ### Destination Addressing Mode (Short Address)
        0, 0,
    ### Frame Version (IEEE 802.15.4)
        0, 0,
    ### Source Addressing Mode (Short Address)
        0, 0,
    ]
    ## Sequence Number (1 byte)
    seq = [ 0, 1, 0, 1, 0, 1, 1, 0 ]
    # MAC Footer (MFR)
    ## Frame Check Sum (FCS) (2 bytes)
    ##### XXX:  233
    fcs = [ 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, ]

    lut = [
        0x744ac39b,
        0x44ac39b7,
        0x4ac39b74,
        0xac39b744,
        0xc39b744a,
        0x39b744ac,
        0x9b744ac3,
        0xb744ac39,
        0xdee06931,
        0xee06931d,
        0xe06931de,
        0x06931dee,
        0x6931dee0,
        0x931dee06,
        0x31dee069,
        0x1dee0693,
    ]

    def setUp (self):
        self.tb = gr.top_block()

    def tearDown (self):
        self.tb = None
    
    def to_bits(self, x, n=32 ):
        bits = []
        for i in range( 0 , n ):
            bits = bits + [ (x >> i) & 1 ]
        return bits

    def to_chips(self, bits):
        chips = []
        for i in range( 0, len( bits ), 4 ):
            nibble = (bits[i+3] << 3) | (bits[i+2] << 2) | (bits[i+1] << 1) | (bits[i+0] << 0)
            new_chips = self.to_bits( self.lut[ nibble ] ) 
            chips = chips + new_chips
        return chips
    
    def buf_to_result (self, buf):
        result = []
        for i in range( 0, len( buf ) ):
            result = result + self.to_bits( buf[ i ], 8 )
        return [ int(i) for i in result ]
    
    def test_002_insanity (self):
        
        expected_lqi = 255 # there is zero noise in the system below
        
        # PHY Service Data Unit (PSDU) bits
        psdu_bits = self.fc + self.seq + self.fcs
        
        psdu_bits_len = len( psdu_bits )
        
        # PHY Protocol Data Unit (PPDU) bits
        # acknowledgement packet as defined in 802.15.4-2011.pdf, pp 57-61, 160
        src_bits = self.preamble + self.sfd + self.frame_length + psdu_bits
        # convert to chips
        src_data = self.to_chips( src_bits )

        expected_result = src_data
        
        expected_len = len( src_bits ) / 4 * 32
        actual_len = len( src_data )
        self.assertEqual( expected_len, actual_len, "expected packet length %d does not match actual packet length %d" % ( expected_len, actual_len ) )
        
        # instantiate blocks
        src = blocks.vector_source_f( src_data )
        psnk = ieee802154.packet_sink( 10 )
        pdu = blocks.tagged_stream_to_pdu( blocks.float_t, "packet_len" )
        snk = blocks.vector_sink_f()
        dbg = blocks.message_debug()
        
        # connect stuff
        self.tb.connect( src, psnk )
        self.tb.msg_connect( psnk, "out", dbg, "store")
        #self.tb.connect( pdu, snk )
        
        # craft our message
        # port = pmt.intern( 'pdus' )
        # msg = pmt.cons( pmt.PMT_NIL, pmt.init_f32vector( 10, src_data ) )
        
        # post message
        # src.to_basic_block()._post( port, msg )
        
        # do simulation
        self.tb.start()
        max_nsleeps = 10
        nsleeps = 0
        while dbg.num_messages() < 1 and nsleeps <= max_nsleeps:
            time.sleep( 0.1 )
            nsleeps = nsleeps + 1
        self.tb.stop()
        self.tb.wait()
        
        self.assertEquals( 1, dbg.num_messages(), "we should have processed exactly 1 message" )

        # fetch data
        result_msg = dbg.get_message( 0 )
        
        metadata = pmt.to_python( pmt.car( result_msg ) )
        
        actual_lqi = metadata[ 'lqi' ]
        self.assertEqual( actual_lqi, expected_lqi, "lqi (%u) should be equal to expected lqi (%u)" % ( actual_lqi, expected_lqi ) )

        buf = pmt.u8vector_elements( pmt.cdr( result_msg ) )
        
        actual_result = self.buf_to_result( buf )
        expected_result = psdu_bits
        
        actual_result_len = len( actual_result )
        expected_result_len = len( expected_result )
        
        self.assertEqual( expected_result_len, actual_result_len )
        # check data
        #actual_result = list( snk.data() )
        
        self.assertEqual( expected_result, actual_result )

if __name__ == '__main__':
    gr_unittest.run(qa_packet_sink, "qa_packet_sink.xml")
