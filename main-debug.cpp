#include <Arduino.h>
#include <soc/sens_reg.h>
#include <soc/rtc_io_reg.h>
#include "sensesp.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"

#define FRAME_SIZE   13
#define BMS_RX_PIN   25
#define DUMMY_TX_PIN -1

class EVPowerMonitorBridge {
  private:
    uint8_t buffer[FRAME_SIZE];
    size_t bufIndex = 0;

  public:
    sensesp::ObservableValue<float>* voltageSensor;
    sensesp::ObservableValue<float>* currentSensor;
    sensesp::ObservableValue<float>* socSensor;
    sensesp::ObservableValue<float>* ahSensor;

    EVPowerMonitorBridge() {
      voltageSensor = new sensesp::ObservableValue<float>();
      currentSensor = new sensesp::ObservableValue<float>();
      socSensor     = new sensesp::ObservableValue<float>();
      ahSensor      = new sensesp::ObservableValue<float>();
    }

    void handle_rx_stream() {
      while (Serial2.available() > 0) {
        uint8_t incomingByte = Serial2.read();

        // 1. Force alignment: Sync array frames strictly on 'R' (0x52)
        if (bufIndex == 0 && incomingByte != 0x52) {
          continue; 
        }

        buffer[bufIndex++] = incomingByte;

        // 2. Validate the 'REC' preamble layout to reject line noise surges
        if (bufIndex == 2 && buffer[1] != 0x45) { bufIndex = 0; continue; } // 'E'
        if (bufIndex == 3 && buffer[2] != 0x43) { bufIndex = 0; continue; } // 'C'

        // 3. Process metrics when a complete 13-byte array finishes compiling
        if (bufIndex >= FRAME_SIZE) {
          parse_bms_frame(buffer);
          bufIndex = 0; 
        }
      }
    }

  private:
    void parse_bms_frame(uint8_t* frame) {
      // --------------------------------------------------------------
      // RAW BMS DISPLAY DATA
      // Print the exact 13-byte frame received from the character display.
      // --------------------------------------------------------------
      Serial.print("[BMS RAW] REC ");
      for (int i = 0; i < FRAME_SIZE; i++) {
          Serial.printf("%02X ", frame[i]);
      }
      Serial.println();
      
      // Also print indexed bytes to make protocol analysis easier.
      Serial.print("[BMS IDX] ");
      for (int i = 0; i < FRAME_SIZE; i++) {
          Serial.printf("[%d]=%02X ", i, frame[i]);
      }
      Serial.println();

      // --------------------------------------------------------------
      // 📊 AUTOMATIC TELEMETRY EXTRACTION ENGINE (PEV 3.1 SPEC COMPLIANT)
      // --------------------------------------------------------------
      
      // 1. Extract Voltage (Indices 3 & 4) - 16-bit Little-Endian, Scale Factor 10
      uint16_t rawVolts = static_cast<uint16_t>(frame[3]) | (static_cast<uint16_t>(frame[4]) << 8);
      float liveVoltage = static_cast<float>(rawVolts) / 10.0f;

      // 2. Extract Amps Sign (Index 5) & Current (Indices 6 & 7) - 16-bit Little-Endian, Scale Factor 10
      uint8_t ampsSign = frame[5]; 
      uint16_t rawCurrent = static_cast<uint16_t>(frame[6]) | (static_cast<uint16_t>(frame[7]) << 8);
      float liveCurrent = static_cast<float>(rawCurrent) / 10.0f;
      
      // Apply discharge sign rules natively from register 5 (1 = discharge)
      if (ampsSign == 1) {
          liveCurrent = -liveCurrent;
      }

      // 3. Extract Ampminutes -> Amp Hours (Indices 8 & 9) - 16-bit Little-Endian, Scale Factor 60
      uint16_t rawAmpMins = static_cast<uint16_t>(frame[8]) | (static_cast<uint16_t>(frame[9]) << 8);
      float liveAh = static_cast<float>(rawAmpMins) / 60.0f;
      
      // Since display reads negative values for capacity drawn, mirror it here:
      liveAh = -liveAh; 
      float coulombsDischarged = liveAh * 3600.0f; 

      // 4. Extract State of Charge (Index 10) - 8-bit Unsigned, 0-100%
      uint8_t rawSOC = frame[10];
      float liveSOC = static_cast<float>(rawSOC) / 100.0f; // Signal K standard (0.0 - 1.0)

      // 5. Extract Extras: Error Flags (Index 11) & Temperature (Index 12)
      uint8_t errorFlags = frame[11];
      uint8_t liveTempC = frame[12];

      // --------------------------------------------------------------
      // 🖥️ REAL-TIME TELEMETRY DECODER OUTPUT
      // --------------------------------------------------------------
      Serial.println("\n--- [DYNAMIC BMS TELEMETRY DECODER - PEV 3.1] ---");
      Serial.printf("  VOLTAGE OUTPUT => %.1f V   (Raw: %u)\n", liveVoltage, rawVolts);
      Serial.printf("  CURRENT OUTPUT => %.1f A   (Raw: %u, Sign Reg: %u)\n", liveCurrent, rawCurrent, ampsSign);
      Serial.printf("  NET AH OUTPUT  => %.1f Ah  (Raw Amp-Mins: %u)\n", liveAh, rawAmpMins);
      Serial.printf("  SOC OUTPUT     => %.0f %%\n", liveSOC * 100.0f);
      Serial.printf("  TEMPERATURE    => %u °C\n", liveTempC);
      Serial.printf("  ERROR FLAGS    => 0x%02X  (%s)\n", errorFlags, (errorFlags == 0) ? "OK" : "ALERT");
      Serial.println("-------------------------------------------------");

      // Emit clean, verified metrics directly to Signal K paths
      voltageSensor->emit(liveVoltage);
      currentSensor->emit(liveCurrent);
      socSensor->emit(liveSOC);
      ahSensor->emit(coulombsDischarged);
  }
};

std::shared_ptr<sensesp::SensESPApp> sensesp_app;
EVPowerMonitorBridge* evMonitorBridge;

void uartReaderTask(void* pvParameters) {
  while (true) {
    if (evMonitorBridge != nullptr) {
      evMonitorBridge->handle_rx_stream();
    }
    vTaskDelay(pdMS_TO_TICKS(2)); 
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\n--- EV Power Monitor Production Driver Active ---");

  sensesp::SensESPAppBuilder builder;
  sensesp_app = builder.set_hostname("ev-motorbank-bridge")
                       ->get_app();

  // Clear analog overrides from digital pin 25 matrix routing
  CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC);
  SET_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC);
  pinMode(BMS_RX_PIN, INPUT);

  // Initialize inverted hardware UART serial stream at 4800 baud
  Serial2.begin(4800, SERIAL_8N1, BMS_RX_PIN, DUMMY_TX_PIN, true);
  Serial2.setRxBufferSize(1024);

  evMonitorBridge = new EVPowerMonitorBridge();

  evMonitorBridge->voltageSensor->connect_to(new sensesp::SKOutputFloat("electrical.batteries.motorBank.voltage"));
  evMonitorBridge->currentSensor->connect_to(new sensesp::SKOutputFloat("electrical.batteries.motorBank.current"));
  evMonitorBridge->socSensor->connect_to(new sensesp::SKOutputFloat("electrical.batteries.motorBank.capacity.stateOfCharge"));
  evMonitorBridge->ahSensor->connect_to(new sensesp::SKOutputFloat("electrical.batteries.motorBank.capacity.dischargeSinceFull"));

  xTaskCreatePinnedToCore(uartReaderTask, "UART_Reader", 4096, NULL, 1, NULL, 0);
}

void loop() {}
