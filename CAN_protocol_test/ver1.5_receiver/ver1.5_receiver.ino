#include <Ethernet.h>

// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

#include <CAN.h>
#include <ros.h>
#include "hce_autoexcavator/packetsFromExcavator.h" // custom message
#include "hce_autoexcavator/packetsToExcavator.h"   // custom message

// Setup a trigger digitalOutput pin & shield settings
#define PIN_TRIGGER 6
#define CS_CANMODULE 3
#define PIN_CAN_INTERRUPT 7

long can_id;

bool flag_sensor1 = false; //sensor1: Arm(32bit),swing(32bit) pressure
bool flag_sensor2 = false; //sensor2: Boom(32bit),Bucket(32bit) pressure
bool flag_sensor3 = false; //sensor3: Body Pitch(24bit), Roll(24bit)
bool flag_sensor4 = false; //sensor4: Boom Pitch(24bit)
bool flag_sensor5 = false; //sensor5: Arm Pitch(24bit)
bool flag_sensor6 = false; //sensor6: Bucket Pitch(24bit)
bool flag_sensor7 = false; //sensor7: Swing Angle(24bit)

// Measure can write speed
unsigned long start;
unsigned long finish;
unsigned long delta;

// Setup a trigger digitalOutput pin & shield settings
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 76);
IPAddress server(192, 168, 1, 1);
const uint16_t serverPort = 11411; // rosserial socket server port, 11411

// timestamp when the trigger signal is fired.
bool trigger_state = false;
volatile unsigned long trigger_time = 0;
unsigned long time_sec = 0;
unsigned long time_nsec = 0;
volatile unsigned long triggerCounter = 0;

// node handler
ros::NodeHandle nh;

// publisher for timestamp
hce_autoexcavator::packetsFromExcavator msg;
ros::Publisher pub_msg("/canpackets/FromExcavator", &msg);

// Subribe: callback function
void callbackToExcavator(const hce_autoexcavator::packetsToExcavator &msg_from_mpc) {
  start = micros();
  CAN.beginExtendedPacket(0x18FFC920); //0.2s Swing angle, (Length: Boom, Arm, Bucket)
  for (int j = 0; j < 8; ++j) {
    CAN.write(msg_from_mpc.bytes[j]);
    //Serial.println(msg_from_mpc.bytes[j]); //test_jh
  }
  CAN.endPacket();
  //delayMicroseconds(300);

  CAN.beginExtendedPacket(0x18FFCA21); //0.2s Swing rate, (Velocity: Boom, Arm, Bucket)
  for (int j = 8; j < 16; ++j) {
    CAN.write(msg_from_mpc.bytes[j]);
    //Serial.println(msg_from_mpc.bytes[j]); //test_jh
  }
  CAN.endPacket();
  //delayMicroseconds(300);

  CAN.beginExtendedPacket(0x18FFCB22); //0.2s Swing Torque, (Force: Boom, Arm, Bucket)
  for (int j = 16; j < 24; ++j) {
    CAN.write(msg_from_mpc.bytes[j]);
    //Serial.println(msg_from_mpc.bytes[j]);
  }
  CAN.endPacket();
  //delayMicroseconds(300);

  CAN.beginExtendedPacket(0x18FFCC50); //0.5s Swing angle, (Length: Boom, Arm, Bucket)
  for (int j = 24; j < 32; ++j) {
    CAN.write(msg_from_mpc.bytes[j]);
  }
  CAN.endPacket();
  //delayMicroseconds(300);

  CAN.beginExtendedPacket(0x18FFCD51); //0.5s Swing rate, (Velocity: Boom, Arm, Bucket)
  for (int j = 32; j < 40; ++j) {
    CAN.write(msg_from_mpc.bytes[j]);
  }
  CAN.endPacket();
  //delayMicroseconds(300);

  CAN.beginExtendedPacket(0x18FFCE52); //0.5s Swing Torque, (Force: Boom, Arm, Bucket)
  for (int j = 40; j < 48; ++j) {
    CAN.write(msg_from_mpc.bytes[j]);
  }
  CAN.endPacket();
  //delayMicroseconds(300);

  CAN.beginExtendedPacket(0x18FFCFA0); //1.0s Swing angle, (Length: Boom, Arm, Bucket)
  for (int j = 48; j < 56; ++j) {
    CAN.write(msg_from_mpc.bytes[j]);
  }
  CAN.endPacket();
  //delayMicroseconds(300);

  CAN.beginExtendedPacket(0x18FFD0A1); //1.0s Swing rate, (Velocity: Boom, Arm, Bucket)
  for (int j = 56; j < 64; ++j) {
    CAN.write(msg_from_mpc.bytes[j]);
  }
  CAN.endPacket();
  //delayMicroseconds(300);

  CAN.beginExtendedPacket(0x18FFD1A2); //1.0s Swing Torque, (Force: Boom, Arm, Bucket)
  for (int j = 64; j < 72; ++j) {
    CAN.write(msg_from_mpc.bytes[j]);
  }
  CAN.endPacket();
  //delayMicroseconds(300);

  finish = micros();
  delta = finish - start;
  //Serial.print("CAN wirte time: ");
  //Serial.println(delta);
}

// subscriber
ros::Subscriber<hce_autoexcavator::packetsToExcavator> sub_msg("/canpackets/ToExcavator", callbackToExcavator);

void setup() {
    delay(1000);

  // Use serial to monitor the process
  Serial.begin(115200);
  Serial.println("arduino starts");
  //CAN
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
  }
  else {
    CAN.setPins(CS_CANMODULE, PIN_CAN_INTERRUPT); // about 200 us ISR signal
    Serial.println("CAN bus is initialized at 500 kbps");
  }
  // register the receive callback
  CAN.onReceive(callbackCANReceive);
  Serial.println("arduino starts2");

  // Connect the Ethernet
  Ethernet.begin(mac, ip);

  // Let some time for the Ethernet Shield to be initialized
  delay(1000);

  Serial.println("");
  Serial.println("Ethernet connection info...");
  Serial.println("IP address: ");
  Serial.println(Ethernet.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("MY IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  // advertise publisher and initialize msg.
  nh.advertise(pub_msg);
  msg.bytes_length = 34;
  msg.bytes = (uint8_t*)realloc(msg.bytes, msg.bytes_length * sizeof(uint8_t));

  // subscribe initialize
  nh.subscribe(sub_msg);
}

uint8_t cnt = 0;
void loop() {

  if (nh.connected()) {// if ROS master is connected,

    trigger_time = micros(); // microseconds
    time_sec  = trigger_time / 1000000;
    time_nsec = (trigger_time - time_sec * 1000000) * 1000;
    msg.header.seq = 0;
    msg.header.stamp.sec  = time_sec;
    msg.header.stamp.nsec = time_nsec;
    msg.header.frame_id = "my_arduino";

    //Serial.print(can_union.bytes[0]);
    // fill data!

    /*msg.bytes[0] = cnt;
      for (int i = 1; i < msg.bytes_length; ++i) {
      msg.bytes[i] = i;
      }
      pub_msg.publish( &msg );*/

    msg.n_bytes = msg.bytes_length;

    ++cnt;
    if (cnt > 255) cnt = 0;

  }
  else {
    //Serial.print("Not connected \n");
  }// When TCP/IP connection is lost!

  nh.spinOnce();
}


void callbackCANReceive(int packetSize) {
  // received a packet
  if (CAN.packetExtended()) {
    Serial.print("extended ");
  }

  if (CAN.packetRtr()) {
    // Remote transmission request, packet contains no data
    // Serial.print("RTR ");
  }

  if (CAN.packetRtr()) {
    Serial.println(CAN.packetDlc());
  } else {
    can_id = CAN.packetId();

    /*int i = 0;
      while (CAN.available()) {
      can_union.bytes[i] = CAN.read(); //read 1 byte only 0xFF
      i++;
      }*/

    if (can_id == 0x18FF4DE7) {
      int i = 0;
      while (CAN.available()) {
        msg.bytes[i] = CAN.read(); //read 1 byte only 0xFF
        i++;
      }
      flag_sensor1 = true ;
      //Serial.print("1");
    }
    else if (can_id == 0x18FF61E7 && flag_sensor1) {
      int i = 8;
      while (CAN.available()) {
        msg.bytes[i] = CAN.read(); //read 1 byte only 0xFF
        i++;
      }
      flag_sensor2 = true ;
      //Serial.print("2");
    }
    else if (can_id == 0x18F02971 && flag_sensor2) {
      int i = 16;
      while (CAN.available()) {
        msg.bytes[i] = CAN.read(); //read 1 byte only 0xFF
        i++;
        if (i == 22) {
          break;
        }
      }
      flag_sensor3 = true ;
      //Serial.print("3");
    }
    else if (can_id == 0x18F02972 && flag_sensor3) {
      int i = 22;
      while (CAN.available()) {
        msg.bytes[i] = CAN.read(); //read 1 byte only 0xFF
        i++;
        if (i == 25) {
          break;
        }
      }
      flag_sensor4 = true ;
      //Serial.print("4");
    }
    else if (can_id == 0x18F02973 && flag_sensor4) {
      int i = 25;
      while (CAN.available()) {
        msg.bytes[i] = CAN.read(); //read 1 byte only 0xFF
        i++;
        if (i == 28) {
          break;
        }
      }
      flag_sensor5 = true ;
      //Serial.print("5");
    }
    else if (can_id == 0x18F02974 && flag_sensor5) {
      int i = 28;
      while (CAN.available()) {
        msg.bytes[i] = CAN.read(); //read 1 byte only 0xFF
        i++;
        if (i == 31) {
          break;
        }
      }
      flag_sensor6 = true ;
      //Serial.print("6");
    }
    else if (can_id == 0x18F02970 && flag_sensor6) {
      int i = 31;
      while (CAN.available()) {
        msg.bytes[i] = CAN.read(); //read 1 byte only 0xFF
        i++;
        if (i == 34) {
          break;
        }
      }
      flag_sensor7 = true ;
      //Serial.print("7");
    }

    //Send message
    if (flag_sensor1 && flag_sensor2 && flag_sensor3 && flag_sensor4 && flag_sensor5 && flag_sensor6 && flag_sensor7) {
      pub_msg.publish( &msg );
      flag_sensor1 = false;
      flag_sensor2 = false;
      flag_sensor3 = false;
      flag_sensor4 = false;
      flag_sensor5 = false;
      flag_sensor6 = false;
      flag_sensor7 = false;

      //Serial.println(msg.bytes[0]);
    }

  }
}
