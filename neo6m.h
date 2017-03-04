#ifndef NEO6M_H
#define NEO6M_H

#include <Arduino.h>

//POSLLH packet structure
//[HEADER, ID   , LENGTH, PAYLOAD, CHECKSUM]
//[0...1 , 2...3, 4...5 , 6...33 , 34...35 ]
//Little Endian

#define _serial Serial4

#if not defined _serial
	#error You need to define a Serial class to use in "neo6m.h"
#endif

class NEO6M {
	public:
		NEO6M(){
			LONGITUDE		= 0.0;
			LATITUDE		= 0.0;
			HEIGHT_OVER_SEA	= 0.0;
			H_ACCURACY		= 0.0;
			V_ACCURACY		= 0.0;
			_pos			= 0;
		}
		
		float	LONGITUDE, LATITUDE;
		float	HEIGHT_OVER_SEA;
		float	H_ACCURACY, V_ACCURACY;
		
		void setup();
		void update();
		void update_parser(byte b);
		bool is_locked(float threshold);
		bool is_locked(int threshold);
		
	private:
		const static byte POSLLH_MESSAGE_SIZE = 36;
		const static byte POSLLH_DATA_BEGIN = 6;
		const static byte POSLLH_DATA_END = 33;
		const static byte POSLLH_DATA_SIZE = 28;
		
		byte _data[POSLLH_DATA_SIZE];
		byte _pos;
		
		void		calcChecksum(byte *payload, byte payloadSize);
		uint32_t	readU4(byte * b);
		int32_t		readS4(byte * b);
		void		sendUBX(byte *msg, byte msgLength);
};
#endif