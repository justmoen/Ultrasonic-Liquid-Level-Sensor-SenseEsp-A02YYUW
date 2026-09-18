#include "mppt_rs485.h"

// ============================================================
// MPPT RS485 UART
// ============================================================
//
// Serial2 is RESERVED for the BMS/display data stream.
//
// MPPT therefore uses Serial1.
//
// This is important because main.cpp initializes the BMS as:
//
//   Serial2.begin(4800, ..., BMS_RX_PIN, ...);
//
// Previously MPPT also used Serial2, which reconfigured the
// BMS UART to 9600 baud on the MPPT pins.
//
// ============================================================

#define MPPT_UART_NUM 1

#define RS485_RX 17
#define RS485_TX 16
#define RS485_DE 4

#define MPPT_BAUD 9600


MPPT_RS485::MPPT_RS485(uint8_t address, uint32_t poll_interval)
    : address_(address),
      poll_interval_(poll_interval) {

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

  // IMPORTANT:
  // MPPT gets its own UART.
  // BMS remains on Serial2.
  serial_ = &Serial1;
}


void MPPT_RS485::begin() {

  pinMode(RS485_DE, OUTPUT);

  // Start in receive mode.
  digitalWrite(RS485_DE, LOW);

  serial_->begin(
      MPPT_BAUD,
      SERIAL_8N1,
      RS485_RX,
      RS485_TX
  );

  // Delay first poll.
  last_poll_ = millis() + 5000;

  ESP_LOGI(
      "MPPT",
      "RS485 initialized: UART%d RX=%d TX=%d DE=%d baud=%d",
      MPPT_UART_NUM,
      RS485_RX,
      RS485_TX,
      RS485_DE,
      MPPT_BAUD
  );
}


void MPPT_RS485::loop() {

  if (millis() - last_poll_ >= poll_interval_) {

    last_poll_ = millis();

    ESP_LOGI("MPPT", "Polling MPPT...");

    poll();
  }
}


void MPPT_RS485::send_command() {

  uint8_t frame[8] = {
      address_,
      0xA3,
      0x01,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00
  };

  uint16_t sum = 0;

  for (int i = 0; i < 7; i++) {
    sum += frame[i];
  }

  frame[7] = sum & 0xFF;


  // Clear any stale bytes from the MPPT UART only.
  while (serial_->available()) {
    serial_->read();
  }


  ESP_LOGI("MAIN", "TX:");

  for (int i = 0; i < 8; i++) {
    ESP_LOGI("MAIN", "%02X", frame[i]);
  }


  // ----------------------------------------------------------
  // Enable RS485 transmitter
  // ----------------------------------------------------------

  digitalWrite(RS485_DE, HIGH);

  delayMicroseconds(200);

  ESP_LOGI("MAIN", "DE=TX");


  serial_->write(frame, 8);

  // Wait until Arduino's serial buffer has been transmitted.
  serial_->flush();

  // At 9600 baud, allow the final byte to completely leave
  // the UART before switching the transceiver back to RX.
  delayMicroseconds(1200);


  // ----------------------------------------------------------
  // Enable RS485 receiver
  // ----------------------------------------------------------

  digitalWrite(RS485_DE, LOW);

  delayMicroseconds(200);

  ESP_LOGI("MAIN", "DE=RX");
}


bool MPPT_RS485::read_response(uint8_t* buffer, size_t len) {

  uint32_t start = millis();

  size_t index = 0;

  ESP_LOGI("MAIN", "RX Begin...");


  while (millis() - start < 1500) {

    if (serial_->available()) {

      uint8_t b = serial_->read();

      ESP_LOGI("MAIN", "RX: %02X", b);


      // Protect against buffer overflow.
      if (index < len) {
        buffer[index++] = b;
      }


      if (index >= len) {

        ESP_LOGI(
            "MAIN",
            "Response Complete"
        );

        return true;
      }
    }

    // Give other FreeRTOS tasks a chance to execute.
    //
    // This is especially important because Signal K,
    // WiFi and the BMS reader are running concurrently.
    vTaskDelay(pdMS_TO_TICKS(1));
  }


  ESP_LOGI("MAIN", "(timeout)");

  return false;
}


void MPPT_RS485::poll() {

  uint8_t response[21];


  send_command();


  if (!read_response(response, 21)) {

    // Error 1 = timeout.
    comm_error->emit(1);

    return;
  }


  // ----------------------------------------------------------
  // Checksum
  // ----------------------------------------------------------

  uint16_t sum = 0;

  for (int i = 0; i < 20; i++) {
    sum += response[i];
  }


  if ((sum & 0xFF) != response[20]) {

    ESP_LOGW(
        "MPPT",
        "Checksum failure: calculated=%02X received=%02X",
        sum & 0xFF,
        response[20]
    );

    // Error 2 = checksum failure.
    comm_error->emit(2);

    return;
  }


  comm_error->emit(0);


  // ----------------------------------------------------------
  // Status / alarms
  // ----------------------------------------------------------

  uint8_t status = response[3];

  bool operating_fault_bit =
      status & 0x01;

  bool battery_overdischarge_bit =
      status & 0x02;

  bool fan_fault_bit =
      status & 0x04;

  bool overtemp_bit =
      status & 0x08;

  bool charge_short_bit =
      status & 0x10;


  operating_fault->emit(
      operating_fault_bit
  );

  battery_overdischarge->emit(
      battery_overdischarge_bit
  );

  fan_fault->emit(
      fan_fault_bit
  );

  over_temperature->emit(
      overtemp_bit
  );

  charge_short_circuit->emit(
      charge_short_bit
  );


  alarm_operating->emit(
      operating_fault_bit
          ? "Operating fault"
          : "normal"
  );

  alarm_battery->emit(
      battery_overdischarge_bit
          ? "Battery overdischarge"
          : "normal"
  );

  alarm_fan->emit(
      fan_fault_bit
          ? "Fan fault"
          : "normal"
  );

  alarm_overtemp->emit(
      overtemp_bit
          ? "Over temperature"
          : "Normal"
  );


  // ----------------------------------------------------------
  // Measurements
  // ----------------------------------------------------------

  uint16_t pv_raw =
      (response[6] << 8) |
      response[7];

  uint16_t batt_raw =
      (response[8] << 8) |
      response[9];

  uint16_t curr_raw =
      (response[10] << 8) |
      response[11];

  uint16_t int_temp_raw =
      (response[12] << 8) |
      response[13];

  uint16_t ext_temp_raw =
      (response[16] << 8) |
      response[17];


  // ----------------------------------------------------------
  // Convert and emit
  // ----------------------------------------------------------

  pv_voltage->emit(
      pv_raw / 10.0
  );

  battery_voltage->emit(
      batt_raw / 100.0
  );

  charge_current->emit(
      curr_raw / 100.0
  );

  internal_temp1->emit(
      int_temp_raw / 10.0
  );

  external_temp1->emit(
      ext_temp_raw / 10.0
  );
}