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
			LONGITUDE_E7	= 0;
			LATITUDE_E7		= 0;
			HEIGHT_OVER_SEA	= 0.0;
			H_ACCURACY		= 0.0;
			V_ACCURACY		= 0.0;
			
			SPEED			= 0.0;
			ANGLE		= 0.0;
			
			LAT_HOME	= 0;
			LON_HOME	= 0;
			HOME_DX = HOME_DY = HOME_DIST = 0.0;
			HOME_ANGLE	= 0.0;

			_home_set = 20;
			
			_pos = 0;
		}
		
		float	LATITUDE, LONGITUDE;
		int32_t	LATITUDE_E7, LONGITUDE_E7;
		float	HEIGHT_OVER_SEA;
		float	H_ACCURACY, V_ACCURACY;
		float	SPEED;
		float	ANGLE;
		
		int32_t	LAT_HOME_E7, LON_HOME_E7;
		float	LAT_HOME, LON_HOME;
		
		float HOME_DX, HOME_DY, HOME_DIST, HOME_ANGLE;
		
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
		
		uint8_t	_data[POSLLH_DATA_SIZE];
		uint8_t	_pos;
		uint8_t	_home_set;
		
		float	_distance_buffer[4];
		uint8_t	_distance_buffer_size;
		
		void		calcChecksum(byte *payload, byte payloadSize);
		uint32_t	readU4(byte * b);
		int32_t		readS4(byte * b);
		float		latitudeToMeters(float lat);
		float		longitudeToMeters(float lon);
		void		sendUBX(byte *msg, byte msgLength);
};
#endif