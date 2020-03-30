
// the following should be moved to a library

#include <Dps310.h>

Dps310 pressure0(0x77);
Dps310 pressure1(0x76);


void setup() {
  // put your setup code here, to run once:

  pressure0.initialize();
  pressure1.initialize();


  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  
  serialHandler();
}

void serialHandler()
{

}
