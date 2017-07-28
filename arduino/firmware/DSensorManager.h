#include "DistanceSensor.h"
#include <XBee.h>

class DSensorManager {

private:
    // Sensor Ports
#define NUM_SENSORS  12
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

    // Sensors arranged starting front to the left
    int sensorPorts[NUM_SENSORS] = {ds12, 
                                      ds6, ds5, ds4, ds3, ds2, ds1, 
                                      ds7, ds8, ds9,ds10, ds11,};
    DistanceSensor * dSensors[NUM_SENSORS];

    // Buffer length for median purposes
    int DIST_BUFFER_LEN = 5;
    // Period of sensor refresh and send info
    unsigned int DSENSE_PERIOD = 17;
    unsigned int TX_PERIOD = 50;
    // Sensors last updated timestamp
    unsigned long lastDSenseUpdate;
    // Timestamp of last time info was sent
    unsigned long lastTXUpdate;

public:
    DSensorManager() {
      for (int s = 0; s < NUM_SENSORS; s++) {
        dSensors[s] = new RawDistanceSensor(sensorPorts[s], DIST_BUFFER_LEN);
      }

      lastDSenseUpdate = 0;
      lastTXUpdate = 0;
    }

    long getPeriod(){
      return min(DSENSE_PERIOD, TX_PERIOD) * 1000;
    }

    void process(XBee &xbee) {
      unsigned long diffDSense = millis() - lastDSenseUpdate;
      if (diffDSense > DSENSE_PERIOD) {
        lastDSenseUpdate = millis();
        for (int s = 0; s < NUM_SENSORS; s++)
          dSensors[s]->getRawValue();
      }

      // Once on a while - Send data
      unsigned long diffTX = millis() - lastTXUpdate;
      if ( diffTX > TX_PERIOD) {
        lastTXUpdate = millis();

        // Read sensors
        uint8_t sensor_vals[NUM_SENSORS * 2];
        for (int i = 0; i < NUM_SENSORS; i++) {
          int val = dSensors[i]->rawMedian();
          sensor_vals[2 * i] = highByte(val);
          sensor_vals[2 * i + 1] = lowByte(val);
        }
        // Send sensor data
        Tx16Request tx = Tx16Request(0xFFFF, sensor_vals, sizeof(sensor_vals));
        xbee.send(tx);

        
      }
    }
};

