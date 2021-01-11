// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <CAN.h>

void setup() {
  Serial.begin(115200);
  //while (!Serial);

  Serial.println("CAN Sender");

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    //while (1);
  }
}

// Measure can write speed
unsigned long start;
unsigned long finish;
unsigned long delta;

uint8_t cnt = 0;
void loop() {

  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  //Serial.println("Sending packet ... ");
  
  start = micros();
  CAN.beginExtendedPacket(0x18FF4DE7);
  CAN.write(cnt); //CAN.write(0x48); // Arm Cylinder Head Pressure 0~15
  CAN.write(0x00);
  ///////////////
  CAN.write(cnt); //CAN.write(0x07); // Arm Cylinder Rod Pressure 16~31
  CAN.write(0x00);
  ///////////////
  CAN.write(cnt); //CAN.write(0x06); // Swing Left Pressure 32~47
  CAN.write(0x00);
  ///////////////
  CAN.write(cnt); //CAN.write(0x01); // Swing Right Pressure 48~63
  CAN.write(0x03);
  CAN.endPacket();
  delay(1); //delayMicroseconds(100);

  CAN.beginExtendedPacket(0x18FF61E7);
  CAN.write(cnt); //CAN.write(0x4A); // Boom Cylinder Head Pressure 0~15
  CAN.write(0x00);
  ///////////////
  CAN.write(cnt); //CAN.write(0x09); // Boom Cylinder Rod Pressure 16~31
  CAN.write(0x00);
  ///////////////
  CAN.write(cnt); //CAN.write(0x08); // Bucket Cylinder Head Pressure 32~47
  CAN.write(0x00);
  ///////////////
  CAN.write(cnt); //CAN.write(0x03); // Bucket Cylinder Rod Pressure 48~63
  CAN.write(0x03);
  CAN.endPacket();
  delay(1); //delayMicroseconds(100);

  CAN.beginExtendedPacket(0x18F02971);
  CAN.write(cnt); //CAN.write(0xAA); // Body Sensor Pitch Angle 0~23
  CAN.write(0xBB); // CCBBAA->159.4661
  CAN.write(0xCC);
  ///////////////
  CAN.write(cnt); //CAN.write(0xDD); // Body Sensor Roll Angle 24~47
  CAN.write(0xEE); // FFEEDD->261.8661
  CAN.write(0xFF);
  ///////////////
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.endPacket();
  delay(1); //delayMicroseconds(100);

  CAN.beginExtendedPacket(0x18F02972);
  CAN.write(cnt); //CAN.write(0xA2); // Boom Sensor Pitch Angle 0~23
  CAN.write(0x3F); // CCBBAA->26.4971
  CAN.write(0x8A);
  ///////////////
  CAN.write(0xFF);
  CAN.write(0xEE);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.endPacket();
  delay(1); //delayMicroseconds(100);

  CAN.beginExtendedPacket(0x18F02973);
  CAN.write(cnt); //CAN.write(0x2A); // Arm Sensor Pitch Angle 0~23
  CAN.write(0x44); // CCBBAA->-43.4675
  CAN.write(0x67);
  ///////////////
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.endPacket();
  delay(1); //delayMicroseconds(100);

  CAN.beginExtendedPacket(0x18F02974);
  CAN.write(cnt); //CAN.write(0xF8); // Bucket Sensor Pitch Angle 0~23
  CAN.write(0x47); // CCBBAA->-239.4377
  CAN.write(0x05);
  ///////////////
  CAN.write(0xFF);
  CAN.write(0xEE);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.endPacket();
  delay(1);

  CAN.beginExtendedPacket(0x18F02970);
  CAN.write(cnt); //CAN.write(0xF8); // Swing Sensor Angle 0~23
  CAN.write(0x47); // CCBBAA->-239.4377
  CAN.write(0x05);
  ///////////////
  CAN.write(0xFF);
  CAN.write(0xEE);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.endPacket();
  delayMicroseconds(500);

  //Check time
  //finish = micros();
  //delta = finish - start;
  //Serial.print("CAN wirte time: ");
  //Serial.println(delta);

  ++cnt;
  if (cnt > 255) cnt = 0;

  //Serial.println("done");
  /*
    // send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
    Serial.print("Sending extended packet ... ");

    CAN.beginExtendedPacket(0xabcdef);
    CAN.write('w');
    CAN.write('o');
    CAN.write('r');
    CAN.write('l');
    CAN.write('d');
    CAN.endPacket();

    Serial.println("done");

    delay(1000)
  */
}
