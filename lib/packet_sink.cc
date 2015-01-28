/*
 * Copyright 2004,2013 Free Software Foundation, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <ieee802154/packet_sink.h>
#include <gnuradio/io_signature.h>
#include <cstdio>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdexcept>
#include <cstring>
#include <gnuradio/blocks/count_bits.h>
#include <iostream>

using namespace gr::ieee802154;

#include <sys/time.h>

static inline uint64_t stamp() {
	struct timeval tv = {};
	gettimeofday( &tv, NULL );
	return tv.tv_sec;
}
static uint64_t then, now;

#define V_( fmt, args... ) \
    do { \
    	now = stamp(); \
        if( d_debug && now - then >= 1 ) { \
        	then = now; \
            fprintf( stderr, fmt "\n", ##args ); \
            fflush( stderr ); \
        } \
    } while ( 0 )

#if 0
#define V1( fmt, args... ) V_( fmt, ##args )
#else
#define V1( fmt, args... )
#endif
#if 1
#define V2( fmt, args... ) V_( fmt, ##args )
#else
#define V2( fmt, args... )
#endif

#define D( fmt, args... ) V_( fmt, ##args )

#define SFD_FRST ( d_sync_vector & 0xf )
#define SFD_LAST ( (d_sync_vector >> 4) & 0xf )

static const unsigned int CHIP_MAPPING[] = {
	0xd9c3522e,
	0xed9c3522,
	0x2ed9c352,
	0x22ed9c35,
	0x522ed9c3,
	0x3522ed9c,
	0xc3522ed9,
	0x9c3522ed,
	0x8c96077b,
	0xb8c96077,
	0x7b8c9607,
	0x77b8c960,
	0x077b8c96,
	0x6077b8c9,
	0x96077b8c,
	0xc96077b8,
};

static const int MAX_PKT_LEN    = 128 -  1; // remove header and CRC
static const int MAX_LQI_SAMPLES = 8; // Number of chip correlation samples to take

class packet_sink_impl : public packet_sink {
public:

void enter_search()
{
	V1("@ enter_search");

	d_state = STATE_SYNC_SEARCH;
	d_shift_reg = 0;
	d_preamble_cnt = 0;
	d_chip_cnt = 0;
	d_packet_byte = 0;
}
    
void enter_have_sync()
{
	V1("@ enter_have_sync");

	d_state = STATE_HAVE_SYNC;
	d_packetlen_cnt = 0;
	d_packet_byte = 0;
	d_packet_byte_index = 0;

	// Link Quality Information
	d_lqi = 0;
	d_lqi_sample_count = 0;
}

void enter_have_header(int payload_len)
{
	D("@ enter_have_header (payload_len = %d)", payload_len);

	d_state = STATE_HAVE_HEADER;
	d_packetlen  = payload_len;
	d_payload_cnt = 0;
	d_packet_byte = 0;
	d_packet_byte_index = 0;
}


unsigned char decode_chips(unsigned int chips){
	int i;
	int best_match = 0xFF;
	int min_threshold = 33; // Matching to 32 chips, could never have a error of 33 chips

	for(i=0; i<16; i++) {
		// FIXME: we can store the last chip
		// ignore the first and last chip since it depends on the last chip.
		unsigned int threshold = gr::blocks::count_bits32((chips & 0x7FFFFFFE) ^ (CHIP_MAPPING[i] & 0x7FFFFFFE));

		if (threshold < min_threshold) {
			best_match = i;
			min_threshold = threshold;
		}
	}

	if (min_threshold < d_threshold) {
		D("[ %2u ] 0x%08x => 0x%08x [ %2u errors ]", best_match, chips, CHIP_MAPPING[ best_match ], min_threshold );
		// LQI: Average number of chips correct * MAX_LQI_SAMPLES
		//
		if (d_lqi_sample_count < MAX_LQI_SAMPLES) {
			d_lqi += 32 - min_threshold;
			d_lqi_sample_count++;
		}

		return (char)best_match & 0xF;
	}

	return 0xFF;
}

int slice(float x) {
	return x > 0 ? 1 : 0;
}

packet_sink_impl(int threshold, bool debug)
  : block ("packet_sink",
		   gr::io_signature::make(1, 1, sizeof(float)),
		   gr::io_signature::make(0, 0, 0)),
    d_threshold(threshold),
	d_debug(debug)
{
	d_sync_vector = 0xa7;

	// Link Quality Information
	d_lqi = 0;
	d_lqi_sample_count = 0;

	V1("syncvec: %x, threshold: %d", d_sync_vector, d_threshold);
	enter_search();

	message_port_register_out(pmt::mp("out"));

}

~packet_sink_impl()
{
}

int general_work(int noutput, gr_vector_int& ninput_items,
			gr_vector_const_void_star& input_items,
			gr_vector_void_star& output_items ) {

	const float *inbuf = (const float*)input_items[0];
        int ninput = ninput_items[0];
	int count=0;
	int i = 0;

	V1(">>> Entering state machine");

	while(count < ninput) {
		switch(d_state) {

		case STATE_SYNC_SEARCH:    // Look for sync vector
			V1("SYNC Search, ninput=%d syncvec=%x", ninput, d_sync_vector);

			while (count < ninput) {

				if(slice(inbuf[count++]))
					d_shift_reg = (d_shift_reg << 1) | 1;
				else
					d_shift_reg = d_shift_reg << 1;

				if(d_preamble_cnt > 0){
					d_chip_cnt = d_chip_cnt+1;
				}

				// The first if block syncronizes to chip sequences.
				if(d_preamble_cnt == 0){
					unsigned int threshold;
					threshold = gr::blocks::count_bits32((d_shift_reg & 0x7FFFFFFE) ^ (CHIP_MAPPING[0] & 0x7FFFFFFE));
					if(threshold < d_threshold) {
						//  fprintf(stderr, "Threshold %d d_preamble_cnt: %d", threshold, d_preamble_cnt);
						//if ((d_shift_reg&0xFFFFFE) == (CHIP_MAPPING[0]&0xFFFFFE)) {
						V1("Found 0 in chip sequence");
						// we found a 0 in the chip sequence
						d_preamble_cnt+=1;
						//fprintf(stderr, "Threshold %d d_preamble_cnt: %d", threshold, d_preamble_cnt);
					}
				} else {
					// we found the first 0, thus we only have to do the calculation every 32 chips
					if(d_chip_cnt == 32){
						d_chip_cnt = 0;

						if(d_packet_byte == 0) {
							if (gr::blocks::count_bits32((d_shift_reg & 0x7FFFFFFE) ^ (CHIP_MAPPING[0] & 0xFFFFFFFE)) <= d_threshold) {
								V2("Found %d 0 in chip sequence", d_preamble_cnt);
								// we found an other 0 in the chip sequence
								d_packet_byte = 0;
								d_preamble_cnt ++;
							} else if (gr::blocks::count_bits32((d_shift_reg & 0x7FFFFFFE) ^ (CHIP_MAPPING[ SFD_FRST ] & 0xFFFFFFFE)) <= d_threshold) {
								V2("Found first nibble of SFD. 0x%08x", d_shift_reg);
								d_packet_byte = SFD_FRST;
							} else {
								// we are not in the synchronization header
								V2("Wrong first nibble of SFD. 0x%08x", d_shift_reg);
								enter_search();
								break;
							}

						} else {
							if (gr::blocks::count_bits32((d_shift_reg & 0x7FFFFFFE) ^ (CHIP_MAPPING[ SFD_LAST ] & 0xFFFFFFFE)) <= d_threshold) {
								d_packet_byte |= SFD_LAST << 4;
								V2("Found sync, 0x%08x", d_packet_byte);
								// found SDF
								// setup for header decode
								enter_have_sync();
								break;
							} else {
								V2("Found second nibble of SFD. 0x%08x", d_shift_reg);
								enter_search();
								break;
							}
						}
					}
				}
			}
			break;

		case STATE_HAVE_SYNC:
			V2("Header Search bitcnt=%d, header=0x%08x", d_headerbitlen_cnt, d_header);

			while (count < ninput) {		// Decode the bytes one after another.
				if(slice(inbuf[count++]))
					d_shift_reg = (d_shift_reg << 1) | 1;
				else
					d_shift_reg = d_shift_reg << 1;

				d_chip_cnt = d_chip_cnt+1;

				if(d_chip_cnt == 32){
					d_chip_cnt = 0;
					unsigned char c = decode_chips(d_shift_reg);
					if(c == 0xFF){
						// something is wrong. restart the search for a sync
						V2("Found a not valid chip sequence! %u", d_shift_reg);

						enter_search();
						break;
					}

					if(d_packet_byte_index == 0){
						d_packet_byte = c;
					} else {
						// c is always < 15
						d_packet_byte |= c << 4;
					}
					d_packet_byte_index = d_packet_byte_index + 1;
					if(d_packet_byte_index%2 == 0){
						// we have a complete byte which represents the frame length.
						int frame_len = d_packet_byte;
						if(frame_len <= MAX_PKT_LEN){
							enter_have_header(frame_len);
						} else {
							enter_search();
						}
						break;
					}
				}
			}
			break;

		case STATE_HAVE_HEADER:
			V2("Packet Build count=%d, ninput=%d, packet_len=%d", count, ninput, d_packetlen);

			while (count < ninput) {   // shift bits into bytes of packet one at a time
				if(slice(inbuf[count++]))
					d_shift_reg = (d_shift_reg << 1) | 1;
				else
					d_shift_reg = d_shift_reg << 1;

				d_chip_cnt = (d_chip_cnt+1)%32;

				if(d_chip_cnt == 0){
					unsigned char c = decode_chips(d_shift_reg);
					if(c == 0xff){
						// something is wrong. restart the search for a sync
						V2("Found a not valid chip sequence! %u", d_shift_reg);

						enter_search();
						break;
					}
					// the first symbol represents the first part of the byte.
					if(d_packet_byte_index == 0){
						d_packet_byte = c;
					} else {
						// c is always < 15
						d_packet_byte |= c << 4;
					}
					//fprintf(stderr, "%d: 0x%x", d_packet_byte_index, c);
					d_packet_byte_index = d_packet_byte_index + 1;
					if(d_packet_byte_index%2 == 0){
						// we have a complete byte
						V2("packetcnt: %d, payloadcnt: %d, payload 0x%x, d_packet_byte_index: %d", d_packetlen_cnt, d_payload_cnt, d_packet_byte, d_packet_byte_index);

						d_packet[d_packetlen_cnt++] = d_packet_byte;
						d_payload_cnt++;
						d_packet_byte_index = 0;

						if (d_payload_cnt >= d_packetlen){	// packet is filled, including CRC. might do check later in here
							unsigned int scaled_lqi = (d_lqi / MAX_LQI_SAMPLES) << 3;
							unsigned char lqi = (scaled_lqi >= 256? 255 : scaled_lqi);

							pmt::pmt_t meta = pmt::make_dict();
							meta = pmt::dict_add(meta, pmt::mp("lqi"), pmt::from_long(lqi));

							//std::memcpy(buf, d_packet, d_packetlen_cnt);
							pmt::pmt_t payload = pmt::make_blob( d_packet, d_packetlen_cnt );

							message_port_pub(pmt::mp("out"), pmt::cons(meta, payload));

							D("Adding message of size %d to queue", d_packetlen_cnt);
							enter_search();
							break;
						}
					}
				}
			}
			break;

		default:
			assert(0);
			break;

		}
	}

	V1("Samples Processed: %d", ninput_items[0]);

    consume(0, ninput_items[0]);

	return 0;
}

private:
	enum {STATE_SYNC_SEARCH, STATE_HAVE_SYNC, STATE_HAVE_HEADER} d_state;

	bool              d_debug;
	unsigned int      d_sync_vector;           // 802.15.4 standard is 4x 0 bytes and 1x0xA7
	unsigned int      d_threshold;             // how many bits may be wrong in sync vector

	unsigned int      d_shift_reg;             // used to look for sync_vector
	int               d_preamble_cnt;          // count on where we are in preamble
	int               d_chip_cnt;              // counts the chips collected

	unsigned int      d_header;                // header bits
	int               d_headerbitlen_cnt;      // how many so far

	unsigned char     d_packet[MAX_PKT_LEN];   // assembled payload
	unsigned char     d_packet_byte;           // byte being assembled
	int               d_packet_byte_index;     // which bit of d_packet_byte we're working on
	int               d_packetlen;             // length of packet
	int               d_packetlen_cnt;         // how many so far
	int               d_payload_cnt;           // how many bytes in payload

	unsigned int      d_lqi;                   // Link Quality Information
	unsigned int      d_lqi_sample_count;

	// FIXME:
	char buf[256];
};

packet_sink::sptr packet_sink::make(unsigned int threshold, bool debug) {
	return gnuradio::get_initial_sptr(new packet_sink_impl(threshold,debug));
}
