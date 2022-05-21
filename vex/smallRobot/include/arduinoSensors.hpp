#ifndef ARDUINO_SENSORS_HPP
#define ARDUINO_SENSORS_HPP

#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include "robot_specifics.h"

#ifdef HAS_MIDDLE_ENCODER
#define ARDUINO_ENCODER_COUNT 3
#else
#define ARDUINO_ENCODER_COUNT 2
#endif

#define ARDUINO_LEFT_ENCODER 0
#define ARDUINO_RIGHT_ENCODER 1
#ifdef HAS_MIDDLE_ENCODER
  #define ARDUINO_MIDDLE_ENCODER 2
#endif

#define ARDUINO_ENCODER_PPR 100.0
#define ARDUINO_PORT 20
#define ARDUINO_BAUD 115200

void arduino_sensors_setup();

class ArduinoEncoder : public okapi::ContinuousRotarySensor {
private:
  std::atomic<int32_t> *currentRawVal;
  int32_t tareOffset;

public:
  ArduinoEncoder(std::atomic<int32_t> *encoder);

  /**
   * Get the current sensor value, in degrees.
   *
   * @return the current sensor value, or `PROS_ERR` on a failure.
   */
  virtual double get() const override;

  /**
   * Reset the sensor to zero.
   *
   * @return `1` on success, `PROS_ERR` on fail
   */
  virtual std::int32_t reset() override;

  /**
   * Get the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller.
   *
   * @return the current sensor value, or `PROS_ERR` on a failure.
   */
  virtual double controllerGet() override;
};

ArduinoEncoder arduino_encoder_create(int index);

#endif /* ARDUINO_SENSORS_HPP */
