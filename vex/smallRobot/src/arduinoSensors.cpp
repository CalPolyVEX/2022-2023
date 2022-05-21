#include "api.h"
#include "okapi/api.hpp"
#include "arduinoSensors.hpp"

#define ARDUINO_PACKET_SIZE (1 + (4 * ARDUINO_ENCODER_COUNT))

pros::Serial arduino(ARDUINO_PORT, ARDUINO_BAUD);

// change the length of this initializer if you change the encoder count.
std::atomic<int32_t> arduino_encoders[ARDUINO_ENCODER_COUNT] = {
#ifdef HAS_MIDDLE_ENCODER
  0,
#endif
  0,
  0
};

ArduinoEncoder arduino_encoder_create(int index) {
  return ArduinoEncoder(&arduino_encoders[index]);
}

void arduino_sensors_update() {
  // allocate space such that we have two bytes per encoder, as well as an extra
  // byte for packet alignment
  uint8_t arduinoVals[ARDUINO_PACKET_SIZE];

  //count the number of packets available
  int packetsAvail = (arduino.get_read_avail() / ARDUINO_PACKET_SIZE);

  if (packetsAvail) {
    //byte alignment; if only one byte is available, the first readings will be misaligned
    int misalignedBytes = 0;
    uint8_t *p = arduinoVals;
    arduino.read(arduinoVals, ARDUINO_PACKET_SIZE);
    packetsAvail --;
    // Example Misalignment:
    // [xx xx xx 0x80 xx xx xx]
    // 3 byte misalignment
    while (*p != 0x80) {
      p ++;
      misalignedBytes ++;
    }
    uint8_t temp;
    for (int i = 0; i < misalignedBytes; i++) {
      arduino.read(&temp, 1);
    }
    //discard extra packets
    while (packetsAvail) {
      arduino.read(arduinoVals, ARDUINO_PACKET_SIZE);
      packetsAvail --;
    }
    //update encoder vals
    for (int i = 0; i < ARDUINO_ENCODER_COUNT; i++) {
      arduino_encoders[i] = *(((int32_t *)(arduinoVals + 1)) + i);
    }
  }
}

void arduino_sensors_loop(void *param) {
  while(1) {
    // Short delay of 1ms, giving this loop a frequency of 1000Hz.
    // This should be fine since the actual update code is pretty quick.
    arduino_sensors_update();
    pros::delay(1);
  }
}

void arduino_sensors_setup() {
  pros::Task arduino_sensor_task(arduino_sensors_loop, NULL,
    "Arduino Sensor IO Task");
}

ArduinoEncoder::ArduinoEncoder(std::atomic<int32_t> *encoder):
  currentRawVal(encoder), tareOffset(0) {
}

double ArduinoEncoder::get() const {
  // No need to worry about 32-bit integer overflow here, since the motors
  // would have to be spinning at 3 million RPM on average during the 2 minute
  // match.
  //
  // We also have to convert the results from 100 PPR to 360 PPR.
  return ((double) (*this->currentRawVal) * 360.0 / ARDUINO_ENCODER_PPR);
}

std::int32_t ArduinoEncoder::reset() {
  tareOffset = -*this->currentRawVal;

  return 1;
}

double ArduinoEncoder::controllerGet() {
  return get();
}
