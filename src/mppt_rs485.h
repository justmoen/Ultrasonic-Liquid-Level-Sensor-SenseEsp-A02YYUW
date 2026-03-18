#ifndef _MPPT_RS485_H_
#define _MPPT_RS485_H_

#include <Arduino.h>
#include <sensesp.h>
#include <sensesp/system/valueproducer.h>

using namespace sensesp;

class MPPT_RS485 : public ValueProducer<float> {
 public:
  MPPT_RS485(uint8_t address, uint32_t poll_interval);

  void begin();
  void loop();

  FloatProducer* pv_voltage;
  FloatProducer* battery_voltage;
  FloatProducer* charge_current;
  FloatProducer* internal_temp1;
  FloatProducer* external_temp1;

  BoolProducer* operating_fault;
  BoolProducer* battery_overdischarge;
  BoolProducer* over_temperature;
  BoolProducer* fan_fault;
  BoolProducer* charge_short_circuit;

  FloatProducer* comm_error;

  StringProducer* alarm_operating;
  StringProducer* alarm_battery;
  StringProducer* alarm_fan;
  StringProducer* alarm_overtemp;

 private:
  uint8_t address_;
  uint32_t poll_interval_;
  uint32_t last_poll_ = 0;

  void poll();
  void send_command();
  bool read_response(uint8_t* buffer, size_t len);

  HardwareSerial* serial_;
};

#endif
