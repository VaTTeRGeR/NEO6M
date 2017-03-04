#include <neo6m.h>

//Remember to set (#define) a Serial object to use in "neo6m.h"
//Example: "#define _serial Serial3" to use Hardware Serial 3
//defining it here does not do anything, you need to open the neo6m.h file of the library and change it there.

//The library handles Serial-port initialization itself, do not use that port in your program!

NEO6M gps;

void setup() {
  gps.setup();                    //initializes the gps.
}

void loop() {
  gps.update();                   //grabs data and reads packet when finished, needs to be called often!

  if(gps.is_locked(5)) {          //output lat/lon if the gps has at least 5 meters lat/lon precision
    Serial.print(gps.LATITUDE, 7);
    Serial.print(", ");
    Serial.println(gps.LONGITUDE, 7);
  } else {
    Serial.print("Not enough precision yet.(");
    Serial.print(gps.H_ACCURACY,2);
    Serial.println("m)");
  }
  
  //chill a bit to not spam the console, but not too long or the serial buffer overflows and you miss some position updates
  delay(100);
}
