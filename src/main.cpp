// ================= SENSOR ENABLE SWITCHES =================
#define ENABLE_MPU
#define ENABLE_TANK
// #define ENABLE_MPPT
// =========================================================


// Boilerplate #includes:
#include "sensesp_app_builder.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/sensors/sensor.h"
#include <HardwareSerial.h>
#include "sensesp/transforms/linear.h"
#include "sensesp/system/system_status_led.h"
#include <sensesp.h>
#include <ReactESP.h>
#include <limits>

// Sensor-specific includes
#ifdef ENABLE_TANK
#include <UltrasonicA02YYUW.h>
#include <Stream.h>
#endif

#ifdef ENABLE_MPPT
#include "mppt_rs485.h"
#endif

#ifdef ENABLE_MPU
#include <Wire.h>
#include <MPU6050.h>
#endif


using namespace sensesp;

class RollingMaxReporter : public Transform<float, float> {
 public:
  explicit RollingMaxReporter(uint32_t report_interval_ms)
      : Transform<float, float>(""), report_interval_ms_(report_interval_ms) {
    reset_();
    event_loop()->onRepeat(report_interval_ms_, [this]() {
      if (has_sample_) {
        this->emit(max_);
      }
      reset_();
    });
  }

  void set(const float& new_value) override {
    if (!has_sample_) {
      has_sample_ = true;
      max_ = new_value;
      return;
    }
    if (new_value > max_) {
      max_ = new_value;
    }
  }

 private:
  void reset_() {
    has_sample_ = false;
    max_ = -std::numeric_limits<float>::infinity();
  }

  const uint32_t report_interval_ms_;
  bool has_sample_ = false;
  float max_ = -std::numeric_limits<float>::infinity();
};


// ================= MPU =================
#ifdef ENABLE_MPU
MPU6050 mpu;
bool mpu_ok = false;

float pitch = 0;
float roll = 0;

#define I2C_SDA 8
#define I2C_SCL 9
#endif


// ================= TANK =================
#ifdef ENABLE_TANK
const byte txPin = 7;
const byte rxPin = 6;

HardwareSerial sensorSerial(1);
UltrasonicA02YYUW sensor(sensorSerial, rxPin, txPin);

float read_level_callback () { 
    sensor.update(); 
    return sensor.getDistance(); 
}

int read_sensor_status () { 
    return sensor.getDistance() > 0 ? 1 : 0;
}
#endif


// ================= MPPT =================
#ifdef ENABLE_MPPT
#define MPPT_ADDRESS 0x01
MPPT_RS485* mppt = nullptr;
#endif



void setup() {

#ifdef ENABLE_TANK
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    sensorSerial.begin(9600, SERIAL_8N1, rxPin, txPin);
    sensor.begin();
#endif

    delay(3000);

    // Logging
    SetupLogging();
    delay(2000);
    ESP_LOGI("ARDUINO", "--- BOOT START ---");


#ifdef ENABLE_MPU
    // MPU setup
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(100);

    mpu.initialize();
    delay(100);

    if (mpu.testConnection()) {
        Serial.println("MPU6050 connected");
        mpu_ok = true;
    } else {
        Serial.println("MPU6050 connection failed");
        mpu_ok = false;
    }
#endif


    // SensESP app
    auto builder = new SensESPAppBuilder();
    auto safe_led = std::make_shared<SystemStatusLed>(0);

    const char* hostname =
#ifdef ENABLE_MPPT
        "SensESP_MPPT";
#else
        "SensESP_MidShip";
#endif

    sensesp_app = builder
        ->set_hostname(hostname)
        ->set_system_status_led(safe_led) 
        ->get_app();

    ESP_LOGI("ARDUINO", "App object created successfully");


    // ================= MPU =================
#ifdef ENABLE_MPU
    auto* pitch_sensor = new RepeatSensor<float>(1000, []() -> float {
        int16_t ax, ay, az;
        if (!mpu_ok) return 0.0f;

        mpu.getAcceleration(&ax, &ay, &az);

        float axf = ax / 16384.0;
        float ayf = ay / 16384.0;
        float azf = az / 16384.0;

        return atan2(azf, axf);
    });

    auto* roll_sensor = new RepeatSensor<float>(1000, []() -> float {
        int16_t ax, ay, az;
        if (!mpu_ok) return 0.0f;

        mpu.getAcceleration(&ax, &ay, &az);

        float axf = ax / 16384.0;
        float ayf = ay / 16384.0;
        float azf = az / 16384.0;

        return atan2(ayf, axf);
    });

    pitch_sensor
        ->connect_to(new RollingMaxReporter(10000))
        ->connect_to(new SKOutputFloat("navigation.attitude.pitch"));

    roll_sensor
        ->connect_to(new RollingMaxReporter(10000))
        ->connect_to(new SKOutputFloat("navigation.attitude.roll"));
#endif


    // ================= TANK =================
#ifdef ENABLE_TANK
    constexpr uint32_t sample_interval_ms = 1000;
    constexpr uint32_t report_interval_ms = 10000;

    auto* tank_level =
        new RepeatSensor<float>(sample_interval_ms, read_level_callback);

    auto* sensor_status =
        new RepeatSensor<int>(report_interval_ms, read_sensor_status);

    auto int_to_bool_function = [](int sensor_status) -> bool {
        return sensor_status == 1;
    };

    auto int_to_bool_transform =
        new LambdaTransform<int, bool>(int_to_bool_function);

    sensor_status 
        ->connect_to(int_to_bool_transform)
        ->connect_to(new SKOutputBool("tanks.fuel.currentLevel.sensorStatus"));

    const char* sk_path = "tanks.fuel.currentLevel";

    const float empty_value = 40.0;
    const float full_value = 0.0;

    const float multiplier = 1.0 / (full_value - empty_value);
    const float offset = -empty_value * multiplier;

    tank_level  
        ->connect_to(new Linear(multiplier, offset))
        ->connect_to(new RollingMaxReporter(report_interval_ms))
        ->connect_to(new SKOutputFloat(sk_path));
#endif


    // ================= MPPT =================
#ifdef ENABLE_MPPT
    mppt = new MPPT_RS485(MPPT_ADDRESS, 30000);
    mppt->begin();

    if (mppt->pv_voltage) {
        mppt->pv_voltage->connect_to(
            new SKOutputFloat("electrical.chargers.motorMPPT.PVVolt"));
    }

    if (mppt->battery_voltage) {
        mppt->battery_voltage->connect_to(
            new SKOutputFloat("electrical.chargers.motorMPPT.BatteryVolt"));
    }

    if (mppt->charge_current) {
        mppt->charge_current->connect_to(
            new SKOutputFloat("electrical.chargers.motorMPPT.ChargeCurrent"));
    }

    if (mppt->internal_temp1) {
        mppt->internal_temp1->connect_to(
            new SKOutputFloat("electrical.chargers.motorMPPT.InternalTemperature1"));
    }

    if (mppt->external_temp1) {
        mppt->external_temp1->connect_to(
            new SKOutputFloat("electrical.chargers.motorMPPT.ExternalTemperature1"));
    }

    if (mppt->alarm_operating) {
        mppt->alarm_operating->connect_to(
            new SKOutputString("notifications.electrical.chargers.motorMPPT.operatingFault"));
    }

    if (mppt->alarm_battery) {
        mppt->alarm_battery->connect_to(
            new SKOutputString("notifications.electrical.chargers.motorMPPT.batteryOverDischarge"));
    }

    if (mppt->alarm_fan) {
        mppt->alarm_fan->connect_to(
            new SKOutputString("notifications.electrical.chargers.motorMPPT.fanFault"));
    }

    if (mppt->alarm_overtemp) {
        mppt->alarm_overtemp->connect_to(
            new SKOutputString("notifications.electrical.chargers.motorMPPT.overTemperature"));
    }
#endif


    sensesp_app->start();
    ESP_LOGI("ARDUINO", "SensESP Started Successfully!");
}



void loop() {
    event_loop()->tick();

#ifdef ENABLE_MPPT
    if (mppt) {
        mppt->loop();
    }
#endif
}
