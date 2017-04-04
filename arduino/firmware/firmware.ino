#include <XBee.h>
#include <SoftwareSerial.h>
#include "DistanceSensor.h"
#include "PoluloMotor.h"

// Xbee objects
SoftwareSerial xbeeserial(0,1);
XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();
uint8_t option = 0;
uint8_t data = 0;

// Motor data
int m1en = 3;
int m2en = 4;
int m3en = 5;
int m4en = 6;

int m1i1 = 7;
int m1i2 = 8;
int m2i1 = 9;
int m2i2 = 10;
int m3i1 = 11;
int m3i2 = 12;
int m4i1 = 13;
int m4i2 = 14;

int m1enc1 = 15;
int m1enc2 = 16;
int m2enc1 = 17;
int m2enc2 = 18;
int m3enc1 = 19;
int m3enc2 = 20;
int m4enc1 = 21;
int m4enc2 = 22;

PoluloMotor * motors[4];

float motor_angles[4] = {60 * M_PI / 180, 120 * M_PI / 180, 300 * M_PI / 180, 240 * M_PI / 180};
float motor_angles_cos[4] = {cos(motor_angles[0]), cos(motor_angles[1]),cos(motor_angles[2]),cos(motor_angles[3])};
float motor_angles_sin[4]= {sin(motor_angles[0]), sin(motor_angles[1]),sin(motor_angles[2]),sin(motor_angles[3])};

float WHEEL_RADIUS = .0330f;

// Control variables
unsigned long lastRXUpdate = 0, lastTXUpdate = 0, lastDSenseUpdate = 0;
uint8_t x_vel_uint = 128, y_vel_uint = 128, t_vel_uint = 128;

// Sensor data
int ds1 = A9;
int ds2 = A10;
int ds3 = A11;
int ds4 = A12;
int ds5 = A13;
int ds6 = A14;
int ds7 = A15;
int ds8 = A16;
int ds9 = A17;
int ds10 = A18;
int ds11 = A19;
int ds12 = A20;

#define NUM_SENSORS  12
int sensorPorts[] = {ds12, ds11, ds10, ds9, ds8, ds7,
                 ds1, ds2, ds3, ds4, ds5, ds6};
RawDistanceSensor * dSensors[NUM_SENSORS];

// Constants
int DIST_BUFFER_LEN = 13;
unsigned int DSENSE_PERIOD = 17;
unsigned int RX_PERIOD = 40;
unsigned int TX_PERIOD = 50;
int CONTROL_PERIOD = 10000;
long XBEE_ADDR = 0x1111;
float MAX_VEL = 20.0f;
float LINEAR_VEL_SCALE = 4 / 128.0f; 
float ANGULAR_VEL_SCALE =  16 * M_PI / 128.0f;

void setup() {
  // Initialize motors
  motors[0] = new PoluloMotor(m1enc1, m1enc2, m1en, m1i1, m1i2, 30.0f);
  motors[1] = new PoluloMotor(m2enc1, m2enc2, m2en, m2i1, m2i2, 30.0f);
  motors[2] = new PoluloMotor(m3enc1, m3enc2, m3en, m3i1, m3i2, 30.0f);
  motors[3] = new PoluloMotor(m4enc1, m4enc2, m4en, m4i1, m4i2, 30.0f);
  
  xbeeserial.begin(57600);
  xbee.setSerial(xbeeserial);

  for (int s = 0; s < NUM_SENSORS; s++){
    dSensors[s] = new RawDistanceSensor(sensorPorts[s], DIST_BUFFER_LEN);
  }

}

void loop() {
  unsigned long wakeuptime = micros();
  
  // Motor control
  for (int m = 0; m < 4; m++)
    motors[m]->pid();

  unsigned long diffDSense = millis() - lastDSenseUpdate;
  if (diffDSense > DSENSE_PERIOD){
    lastDSenseUpdate = millis();
    for (int s = 0; s < NUM_SENSORS; s++)
      dSensors[s]->getRawValue();
  }
  
  // Once on a while - Send data
  unsigned long diffTX = millis() - lastTXUpdate;
  if ( diffTX > TX_PERIOD){
    lastTXUpdate = millis();

    // Read sensors
    uint8_t sensor_vals[NUM_SENSORS * 2];
    for (int i = 0; i < NUM_SENSORS; i++){
      int val = dSensors[i]->rawMedian();
      sensor_vals[2*i] = highByte(val);
      sensor_vals[2*i+1] = lowByte(val);
    }
    // Send sensor data
    Tx16Request tx = Tx16Request(0xFFFF, sensor_vals, sizeof(sensor_vals));
    xbee.send(tx);

    
    // Receive motor commands
    // Get all pending packages
    xbee.readPacket();
    while (xbee.getResponse().isAvailable()){
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE ) {
        // got a rx packet
        xbee.getResponse().getRx16Response(rx16);
        // get vels from packet
        x_vel_uint = rx16.getData(0);
        y_vel_uint = rx16.getData(1);
        t_vel_uint = rx16.getData(2);
      }
      xbee.readPacket();
    }
  }

  // Once on a while - receive data.
  unsigned long diffRX = millis() - lastRXUpdate;
  if (diffRX > RX_PERIOD){
    lastRXUpdate = millis();
    // Set velocities with values obtained
    // 128 means zero vel
    float x_vel = (x_vel_uint - 128) * LINEAR_VEL_SCALE;
    float y_vel = (y_vel_uint - 128) * LINEAR_VEL_SCALE;
    float t_vel = (t_vel_uint - 128) * ANGULAR_VEL_SCALE;
    
    // precompute rotational component - the same for all wheels
    float rot_comp =  WHEEL_RADIUS * t_vel;
    float maxVel = 0;
    for (int m = 0; m < 4; m++){
      // get precomputed sin and cos for each motor
    //        Serial.print(motor_angles_sin[m]);
    //        Serial.print(" ");
      float cosangle = motor_angles_cos[m];
      float sinangle = motor_angles_sin[m];
      float vel = -sinangle * x_vel + cosangle * y_vel + rot_comp;
      motors[m]->setTargetVel(vel);
      if (abs(vel) > maxVel)
        maxVel = abs(vel);
    }
    
    // Check if we have to normalize
    if (maxVel > MAX_VEL){
      float normalizer = MAX_VEL / maxVel;
      for (int m = 0; m < 4; m++){
         motors[m]->setTargetVel(
          motors[m]->getTargetVel()/normalizer);
      }
    }


  }


  delayMicroseconds(CONTROL_PERIOD - (micros() - wakeuptime));
}
