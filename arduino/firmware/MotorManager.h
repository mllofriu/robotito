#include "PoluloMotor.h"
#include <XBee.h>

class MotorManager {

  private:
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
    float motor_angles_cos[4] = {cos(motor_angles[0]), cos(motor_angles[1]), cos(motor_angles[2]), cos(motor_angles[3])};
    float motor_angles_sin[4] = {sin(motor_angles[0]), sin(motor_angles[1]), sin(motor_angles[2]), sin(motor_angles[3])};

    float WHEEL_RADIUS = (.030f);
    float WHEEL_CIRCUMF = (2*M_PI*WHEEL_RADIUS);
    float ROBOT_RADIUS = 0.03f;


    unsigned int RX_PERIOD = 40;
    unsigned long CONTROL_PERIOD = 10000;
    unsigned long CMD_TIMEOUT = RX_PERIOD*20;
    unsigned long RELEASE_TIMEOUT = RX_PERIOD * 100;
    // Timestamp of last time info was sent
    unsigned long lastRXUpdate;

    float MAX_VEL = 80.0f;
    // 100 means 5 m/s
    float LINEAR_VEL_SCALE = 1 / 100.0f;
    // 100 means 4 turns per second
    float ANGULAR_VEL_SCALE =  2 * M_PI / 100.0f;

    uint8_t x_vel_uint = 128, y_vel_uint = 128, t_vel_uint = 128;

    void setVels(unsigned int x_vel_uint,unsigned int y_vel_uint,unsigned int t_vel_uint) {
      // 128 means zero vel
      float x_vel_signed = (((int)x_vel_uint) - 128);
      float y_vel_signed = (((int)y_vel_uint) - 128); 
      float t_vel_signed = (((int)t_vel_uint) - 128);
     
      // Cap on 100 
      x_vel_signed = x_vel_signed > 100 ? 100 : x_vel_signed;
      x_vel_signed = x_vel_signed < -100 ? -100 : x_vel_signed;
      y_vel_signed = y_vel_signed > 100 ? 100 : y_vel_signed;
      y_vel_signed = y_vel_signed < -100 ? -100 : y_vel_signed;
      t_vel_signed = t_vel_signed > 100 ? 100 : t_vel_signed;
      t_vel_signed = t_vel_signed < -100 ? -100 : t_vel_signed;



      // Convert to m/s and r/s
      float x_vel = x_vel_signed * LINEAR_VEL_SCALE;
      float y_vel = y_vel_signed * LINEAR_VEL_SCALE;
      float t_vel = t_vel_signed * ANGULAR_VEL_SCALE;
      // precompute rotational component - the same for all wheels
      float rot_comp =  ROBOT_RADIUS * t_vel;
      float maxVel = 0;
      for (int m = 0; m < 4; m++) {
        // get precomputed sin and cos for each motor
        //        Serial.print(motor_angles_sin[m]);
        //        Serial.print(" ");
        float cosangle = motor_angles_cos[m];
        float sinangle = motor_angles_sin[m];
        float vel = -(-sinangle * x_vel + cosangle * y_vel + rot_comp) * (2 * M_PI / WHEEL_CIRCUMF);
        motors[m]->setTargetVel(vel);
        if (abs(vel) > maxVel)
          maxVel = abs(vel);
      }

      // Check if we have to normalize
      if (maxVel > MAX_VEL) {
        float normalizer = MAX_VEL / maxVel;
        for (int m = 0; m < 4; m++) {
          motors[m]->setTargetVel(
            motors[m]->getTargetVel() / normalizer);
        }
      }
    }

  public:
    MotorManager() {
      // Initialize motors
      motors[0] = new PoluloMotor(m1enc1, m1enc2, m1en, m1i1, m1i2, 30.0f);
      motors[1] = new PoluloMotor(m2enc1, m2enc2, m2en, m2i1, m2i2, 30.0f);
      motors[2] = new PoluloMotor(m3enc1, m3enc2, m3en, m3i1, m3i2, 30.0f);
      motors[3] = new PoluloMotor(m4enc1, m4enc2, m4en, m4i1, m4i2, 30.0f);

//      setVels(130,128,128);
      lastRXUpdate = millis();
    }

    long getPeriod(){
      return min(CONTROL_PERIOD, RX_PERIOD * 1000);
    }

    void process(XBee &xbee) {
      // Receive motor commands
      unsigned long diffRX = millis() - lastRXUpdate;
      

      if (diffRX > RX_PERIOD) {
        // Get all pending packages
        // Receive packet
        xbee.readPacket();
        while (xbee.getResponse().isAvailable()) {
          if (xbee.getResponse().getApiId() == RX_16_RESPONSE ) {
            // Got a rx packet
//            Serial.println("got packet");
            Rx16Response rx16;
            xbee.getResponse().getRx16Response(rx16);
            // 'v' means set velocities
            if (rx16.getData(0) == 'v'){
              // get vels from packet
              setVels(rx16.getData(1), rx16.getData(2), rx16.getData(3));
            } else if (rx16.getData(0) == 'k'){
              for (int m = 0; m < 4; m++){
                motors[m]->setPID(rx16.getData(3*m+1),rx16.getData(3*m+2),rx16.getData(3*m+3));
              }
            } else if (rx16.getData(0) == 'r'){
              for (int m = 0; m < 4; m++) 
                motors[m]->releaseMotor();
            } else if (rx16.getData(0) == 'e'){
              for (int m = 0; m < 4; m++) 
                motors[m]->engageMotor();
            }
            lastRXUpdate = millis();
          }
          xbee.readPacket();
        }
      }

      // If we havent heard from the boss from a while - do nothing
      if (millis() - lastRXUpdate > CMD_TIMEOUT){
        setVels(128,128,128);

      }

      if (millis() - lastRXUpdate > RELEASE_TIMEOUT){
        for (int m = 0; m < 4; m++) 
          motors[m]->releaseMotor();
      }
      
      // Motor control
      for (int m = 0; m < 4; m++)
        motors[m]->pid();

//      Serial.println(motors[0]->getTargetVel() - motors[0]->getVel());
    }
};

