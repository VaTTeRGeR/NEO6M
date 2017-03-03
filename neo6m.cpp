#include <neo6m.h>

void NEO6M::setup() {
	NEO6M_Serial.begin(9600);

	delay(25);
  
	byte prt[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x72};
	calcChecksum(&prt[2], sizeof(prt)-4);
	sendUBX(&prt[0], sizeof(prt));

	delay(25);
  
	NEO6M_Serial.end();
	NEO6M_Serial.begin(115200);
  
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
  while (NEO6M_Serial.available() > 0) {
    update_parser(NEO6M_Serial.read());
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
		LONGITUDE		= readS4(_data + 4);	//Longitude * 10^+7	[deg]
		LATITUDE		= readS4(_data + 8);	//Latitude  * 10^+7	[deg]
		HEIGHT_OVER_SEA	= readS4(_data + 16);	//Height above mean sea-level	[mm]
		H_ACCURACY		= readU4(_data + 20);	//Horizontal accuracy			[mm]
		V_ACCURACY		= readU4(_data + 24);	//Vertical accuracy				[mm]
		
        _pos = 0;
      }
    break;
  }
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
		NEO6M_Serial.write(msg[i]);
		NEO6M_Serial.flush();
	}
	NEO6M_Serial.println();
	NEO6M_Serial.flush();
}