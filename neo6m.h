#ifndef NEO6M_H
#define NEO6M_H

#include <Arduino.h>

//POSLLH packet structure
//[HEADER, ID   , LENGTH, PAYLOAD, CHECKSUM]
//[0...1 , 2...3, 4...5 , 6...33 , 34...35 ]
//Little Endian

#if not defined (NEO6M_Serial)
	#define NEO6M_Serial Serial4
#endif

class NEO6M {
	public:
		NEO6M(){
			LONGITUDE		= 0;
			LATITUDE		= 0;
			HEIGHT_OVER_SEA	= 0;
			H_ACCURACY		= 0;
			V_ACCURACY		= 0;
			_pos			= 0;
		}
		
		int32_t		LONGITUDE, LATITUDE, HEIGHT_OVER_SEA;
		uint32_t	H_ACCURACY, V_ACCURACY;
		
		void setup();
		void update();
		void update_parser(byte b);
		
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