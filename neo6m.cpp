#include <neo6m.h>

void NEO6M::setup() {
	_serial.begin(9600);

	delay(25);
  
	byte prt[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x72};
	calcChecksum(&prt[2], sizeof(prt)-4);
	sendUBX(&prt[0], sizeof(prt));

	delay(25);
  
	_serial.end();
	_serial.begin(115200);
  
	delay(25);

	byte posllh[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x17, 0xCD};
	calcChecksum(&posllh[2], sizeof(posllh)-4);
	sendUBX(&posllh[0], sizeof(posllh));

	delay(25);
  
	byte nav5[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0xDC};
	calcChecksum(&nav5[2], sizeof(nav5)-4);
	sendUBX(&nav5[0], sizeof(nav5));

	delay(25);
	
	byte rate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96};
	calcChecksum(&rate[2], sizeof(rate)-4);
	sendUBX(&rate[0], sizeof(rate));

	delay(25);

	byte gnss[] = {0xB5, 0x62, 0x06, 0x3E, 0x24, 0x00, 0x00, 0x00, 0x16, 0x04, 0x00, 0x04, 0xFF, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x06, 0x08, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x01, 0xA4, 0x25};
	calcChecksum(&gnss[2], sizeof(gnss)-4);
	sendUBX(&gnss[0], sizeof(gnss));
}

void NEO6M::update() {
	while (_serial.available() > 0) {
		update_parser(_serial.read());
	}
}

void NEO6M::update_parser(byte b) {
	switch (_pos) {
		case 0:
			if(b == 0xB5) _pos++;
			else _pos = 0;
		break;

		case 1:
			if(b == 0x62) _pos++;
			else _pos = 0;
		break;

		case 2:
			if(b == 0x01) _pos++;
			else _pos = 0;
		break;

		case 3:
			if(b == 0x02) _pos++;
			else _pos = 0;
		break;

		case 4:
			if(b == 28) _pos++;
			else _pos = 0;
		break;

		case 5:
			if(b == 0) _pos++;
			else _pos = 0;
		break;

		default:
			if(_pos >= POSLLH_DATA_BEGIN && _pos <= POSLLH_DATA_END) {
				_data[_pos - POSLLH_DATA_BEGIN] = b;
				_pos++;
			} else {
				LONGITUDE_E7	= readS4(_data + 4);					//Longitude						[deg*10e7]
				LATITUDE_E7		= readS4(_data + 8);					//Latitude						[deg*10e7]
				HEIGHT_OVER_SEA	= ((float)readU4(_data + 16))/1000.0;	//Height above mean sea-level	[m]
				H_ACCURACY		= ((float)readU4(_data + 20))/1000.0;	//Horizontal accuracy			[m]
				V_ACCURACY		= ((float)readU4(_data + 24))/1000.0;	//Vertical accuracy				[m]

				float lat_tmp = LATITUDE;
				float lon_tmp = LONGITUDE;
				
				LATITUDE	= ((float)LATITUDE_E7) /10000000.0;		//Latitude						[deg]
				LONGITUDE	= ((float)LONGITUDE_E7)/10000000.0;		//Longitude						[deg]
				
				lat_tmp = latitudeToMeters (LATITUDE  - lat_tmp);
				lon_tmp = longitudeToMeters(LONGITUDE - lon_tmp);
				
				float speed_new = sqrt(lat_tmp*lat_tmp + lon_tmp*lon_tmp) * 4/*HZ*/ * 3.6; //[km/h]

				SPEED = 0.25*SPEED + 0.75*speed_new;
				ANGLE = atan2(lon_tmp, lat_tmp) * 180.0/PI;
				
				if(_home_set > 0 && is_locked(3.0)) {
					if(LAT_HOME_E7 == 0 && LON_HOME_E7 == 0) {
						LAT_HOME_E7 = LATITUDE_E7;
						LON_HOME_E7 = LONGITUDE_E7;
					} else {
						LAT_HOME_E7 = (LAT_HOME_E7*3)/4 + LATITUDE_E7 /4 + 1;
						LON_HOME_E7 = (LON_HOME_E7*3)/4 + LONGITUDE_E7/4 + 1;
						
						LAT_HOME = ((float)LAT_HOME_E7)/10000000.0;
						LON_HOME = ((float)LON_HOME_E7)/10000000.0;
						
						_home_set--;
					}
				} else if(_home_set == 0){
					float d_lat = LATITUDE  - LAT_HOME;
					float d_lon = LONGITUDE - LON_HOME;
					
					HOME_ANGLE = atan2(d_lon, d_lat) * 180.0/PI;
					
					HOME_DX = longitudeToMeters(d_lon);
					HOME_DY = latitudeToMeters(d_lat);
					
					HOME_DIST = sqrt(HOME_DX*HOME_DX + HOME_DY*HOME_DY);
				}
				
				_pos = 0;
			}
		break;
	}
}

bool NEO6M::is_locked(float threshold) {
	return (H_ACCURACY > 0.0 && H_ACCURACY < threshold);
}

bool NEO6M::is_locked(int threshold) {
	return (H_ACCURACY > 0.0 && H_ACCURACY < ((float)threshold));
}


float NEO6M::latitudeToMeters(float dlat) {
	return 111000.0 * dlat;
}

float NEO6M::longitudeToMeters(float dlon) {
	return HOME_DX = 111000.0 * dlon * cos(LATITUDE * PI/180.0);
}

uint32_t NEO6M::readU4(byte * b) {
	uint32_t value = (uint32_t)(*b);  b++;
	value |= ((uint32_t)(*b)) << 8;    b++;
	value |= ((uint32_t)(*b)) << 16;   b++;
	value |= ((uint32_t)(*b)) << 24;
	return value;
}

int32_t NEO6M::readS4(byte * b) {
	int32_t value = (int32_t)(*b);  b++;
	value |= ((int32_t)(*b)) << 8;    b++;
	value |= ((int32_t)(*b)) << 16;   b++;
	value |= ((int32_t)(*b)) << 24;
	return value;
}

void NEO6M::calcChecksum(byte *payload, byte payloadSize) {
	byte CK_A = 0, CK_B = 0;
	for (int i = 0; i < payloadSize ;i++) {
		CK_A = CK_A + *payload;
		CK_B = CK_B + CK_A;
		payload++;
	}
	*payload = CK_A;
	payload++;
	*payload = CK_B;
}

void NEO6M::sendUBX(byte *msg, byte msgLength) {
	for(int i = 0; i < msgLength; i++) {
		_serial.write(msg[i]);
		_serial.flush();
	}
	_serial.println();
	_serial.flush();
}
