#include "mppt_rs485.h"

#define RS485_RX 17
#define RS485_TX 16
#define RS485_DE 4
#define MPPT_BAUD 9600

MPPT_RS485::MPPT_RS485(uint8_t address, uint32_t poll_interval)
    : address_(address), poll_interval_(poll_interval) {

  pv_voltage = new FloatProducer();
  battery_voltage = new FloatProducer();
  charge_current = new FloatProducer();
  internal_temp1 = new FloatProducer();
  external_temp1 = new FloatProducer();

  operating_fault = new BoolProducer();
  battery_overdischarge = new BoolProducer();
  over_temperature = new BoolProducer();
  fan_fault = new BoolProducer();
  charge_short_circuit = new BoolProducer();

  comm_error = new FloatProducer();

  alarm_operating = new StringProducer();
  alarm_battery = new StringProducer();
  alarm_fan = new StringProducer();
  alarm_overtemp = new StringProducer();

  serial_ = &Serial2;
}

void MPPT_RS485::begin() {
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);  // start in RX mode
  serial_->begin(MPPT_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  last_poll_ = millis() + 5000;  // delay first poll only
}

void MPPT_RS485::loop() {

  if (millis() - last_poll_ >= poll_interval_) {
      last_poll_ = millis();
      ESP_LOGI("MPPT", "Polling MPPT...");
      poll();
  }
}

void MPPT_RS485::send_command() {
  uint8_t frame[8] = {address_, 0xA3, 0x01, 0, 0, 0, 0, 0};

  uint16_t sum = 0;
  for (int i = 0; i < 7; i++) sum += frame[i];
  frame[7] = sum & 0xFF;

  // clear buffer
  while (serial_->available()) {
    ESP_LOGI("MAIN", "RX: %02X", serial_->read());
  }

ESP_LOGI("MAIN", "TX: ");
  for (int i = 0; i < 8; i++) {
    ESP_LOGI("MAIN", "%02X", frame[i]);
  }
  ESP_LOGI("MAIN", "");

  // TX mode
  digitalWrite(RS485_DE, HIGH);  // Enable TX
  delayMicroseconds(200);        // Settle

  ESP_LOGI("MAIN", "DE=TX");

  serial_->write(frame, 8);
  serial_->flush();              // Wait TX complete

  digitalWrite(RS485_DE, LOW);   // Back to RX
  delayMicroseconds(200);        // Settle

  ESP_LOGI("MAIN", "DE=RX");
  delay(10);                     // Allow reply
}

bool MPPT_RS485::read_response(uint8_t* buffer, size_t len) {
  uint32_t start = millis();     // Fix scope
  size_t index = 0;

  ESP_LOGI("MAIN", "RX: ");

  while (millis() - start < 1500) {
    if (serial_->available()) {
      uint8_t b = serial_->read();
      ESP_LOGI("MAIN", "RX: %02X", b);

      buffer[index++] = b;
      if (index >= len) {
        ESP_LOGI("MAIN", "");
        return true;
      }
    }
  }

  ESP_LOGI("MAIN", " (timeout)");
  return false;
}

void MPPT_RS485::poll() {
  uint8_t response[21];

  send_command();

  if (!read_response(response, 21)) {
    comm_error->emit(1);
    return;
  }

  uint16_t sum = 0;
  for (int i = 0; i < 20; i++) sum += response[i];
  if ((sum & 0xFF) != response[20]) {
    comm_error->emit(2);
    return;
  }

  comm_error->emit(0);

  uint8_t status = response[3];

  bool operating_fault_bit = status & 0x01;
  bool battery_overdischarge_bit = status & 0x02;
  bool fan_fault_bit = status & 0x04;
  bool overtemp_bit = status & 0x08;
  bool charge_short_bit = status & 0x10;

  operating_fault->emit(operating_fault_bit);
  battery_overdischarge->emit(battery_overdischarge_bit);
  fan_fault->emit(fan_fault_bit);
  over_temperature->emit(overtemp_bit);
  charge_short_circuit->emit(charge_short_bit);

  alarm_operating->emit(operating_fault_bit ? "Operating fault" : "normal");
  alarm_battery->emit(battery_overdischarge_bit ? "Battery overdischarge" : "normal");
  alarm_fan->emit(fan_fault_bit ? "Fan fault" : "normal");
  alarm_overtemp->emit(overtemp_bit ? "Over temperature" : "Normal");

  uint16_t pv_raw        = (response[6]  << 8) | response[7];
  uint16_t batt_raw      = (response[8]  << 8) | response[9];
  uint16_t curr_raw      = (response[10] << 8) | response[11];
  uint16_t int_temp_raw  = (response[12] << 8) | response[13];
  uint16_t ext_temp_raw  = (response[16] << 8) | response[17];

  pv_voltage->emit(pv_raw / 10.0);
  battery_voltage->emit(batt_raw / 100.0);
  charge_current->emit(curr_raw / 100.0);
  internal_temp1->emit(int_temp_raw / 10.0);
  external_temp1->emit(ext_temp_raw / 10.0);
}
