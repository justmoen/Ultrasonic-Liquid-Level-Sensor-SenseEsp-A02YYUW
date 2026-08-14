// ================= SENSOR ENABLE SWITCHES =================
#if !defined(ENABLE_MPU)
#define ENABLE_MPU
#endif

#if !defined(ENABLE_TANK) && !defined(ENABLE_MPPT)
#define ENABLE_TANK
#endif
// =========================================================

// Boilerplate #includes:
#include "sensesp_app_builder.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/sensors/constant_sensor.h"
#include <HardwareSerial.h>
#include "sensesp/transforms/linear.h"
#include "sensesp/system/system_status_led.h"
#include <sensesp.h>
#include <ReactESP.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WebServer.h>
#include <algorithm>
#include <limits>
#include "firmware_version.h"

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

namespace {
String extract_json_string(const String& json, const String& key) {
    String needle = "\"" + key + "\"";
    int key_pos = json.indexOf(needle);
    if (key_pos < 0) {
        return "";
    }

    int value_pos = json.indexOf(':', key_pos);
    if (value_pos < 0) {
        return "";
    }

    int start = json.indexOf('"', value_pos + 1);
    if (start < 0) {
        return "";
    }

    int end = json.indexOf('"', start + 1);
    if (end < 0) {
        return "";
    }

    return json.substring(start + 1, end);
}

String find_asset_url_for_variant(const String& json, const String& variant) {
    int search_pos = 0;
    while (search_pos >= 0) {
        int pos = json.indexOf("\"browser_download_url\"", search_pos);
        if (pos < 0) {
            return "";
        }

        int colon_pos = json.indexOf(':', pos);
        if (colon_pos < 0) {
            return "";
        }

        int start = json.indexOf('"', colon_pos + 1);
        int end = json.indexOf('"', start + 1);
        if (start < 0 || end < 0) {
            return "";
        }

        String candidate = json.substring(start + 1, end);
        if (candidate.endsWith(".bin") && (variant.length() == 0 || candidate.indexOf(variant) >= 0 || candidate.indexOf("latest") >= 0)) {
            return candidate;
        }

        search_pos = end + 1;
    }

    return "";
}

bool install_update_from_url(const String& url) {
    WiFiClientSecure client;
    HTTPClient http;

    if (!http.begin(client, url)) {
        return false;
    }

    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    int http_code = http.GET();
    if (http_code != HTTP_CODE_OK) {
        http.end();
        return false;
    }

    int content_length = http.getSize();
    if (content_length <= 0) {
        http.end();
        return false;
    }

    if (!Update.begin(content_length)) {
        http.end();
        return false;
    }

    WiFiClient* stream = http.getStreamPtr();
    size_t total_written = 0;
    uint8_t buffer[512];
    while (http.connected() && total_written < static_cast<size_t>(content_length)) {
        size_t available = stream->available();
        if (available == 0) {
            delay(1);
            continue;
        }

        size_t chunk_size = std::min<size_t>(sizeof(buffer), std::max<size_t>(available, 1U));
        int bytes_read = stream->readBytes(reinterpret_cast<char*>(buffer), chunk_size);
        if (bytes_read <= 0) {
            break;
        }

        if (Update.write(buffer, bytes_read) != bytes_read) {
            break;
        }

        total_written += static_cast<size_t>(bytes_read);
    }

    bool success = total_written == static_cast<size_t>(content_length) && Update.end(true);
    http.end();

    if (success) {
        ESP_LOGI("ARDUINO", "Firmware update installed successfully");
        ESP.restart();
        return true;
    }

    Update.abort();
    return false;
}

bool check_for_firmware_update(bool force_update = false) {
    String release_url = String("https://api.github.com/repos/") + String(FIRMWARE_REPO_OWNER) + "/" + String(FIRMWARE_REPO_NAME) + "/releases/latest";
    WiFiClientSecure client;
    HTTPClient http;

    if (!http.begin(client, release_url)) {
        return false;
    }

    http.addHeader("Accept", "application/vnd.github+json");
    http.addHeader("User-Agent", "MidShip-ESP32-SignalK");

    int http_code = http.GET();
    if (http_code != HTTP_CODE_OK) {
        http.end();
        return false;
    }

    String payload = http.getString();
    http.end();

    String latest_tag = extract_json_string(payload, "tag_name");
    String latest_asset_url = find_asset_url_for_variant(payload, String(FIRMWARE_VARIANT));

    if (latest_tag.length() == 0 || latest_asset_url.length() == 0) {
        return false;
    }

    if (!force_update && latest_tag == String(FIRMWARE_VERSION)) {
        return false;
    }

    ESP_LOGI("ARDUINO", "New firmware available: %s (%s)", latest_tag.c_str(), latest_asset_url.c_str());
    return install_update_from_url(latest_asset_url);
}

WebServer firmware_server(8081);

void handle_firmware_status() {
    String payload = String("{\"variant\":\"") + String(FIRMWARE_VARIANT) +
                     "\",\"version\":\"" + String(FIRMWARE_VERSION) +
                     "\",\"wifi\":" + (WiFi.status() == WL_CONNECTED ? "true" : "false") +
                     "}";
    firmware_server.sendHeader("Access-Control-Allow-Origin", "*");
    firmware_server.send(200, "application/json", payload);
}

void handle_firmware_update() {
    if (WiFi.status() != WL_CONNECTED) {
        firmware_server.send(200, "application/json", "{\"status\":\"wifi-disconnected\"}");
        return;
    }

    bool update_started = check_for_firmware_update(true);
    firmware_server.send(200, "application/json",
                         String("{\"status\":\"") +
                         (update_started ? "update-started" : "no-update") +
                         "\"}");
}
}

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

    // Transmit yaw as a constant 0 for now (placeholder)
    auto* yaw_constant = new ConstantSensor<float>(0.0f, 10, "/Sensors/Yaw");
    yaw_constant->connect_to(new SKOutputFloat("navigation.attitude.yaw"));
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
        ->connect_to(new SKOutputBool("tanks.fuel.0.sensorStatus"));

    // Send tank capacity as a constant value (0.02) with units metadata.
    // This value is editable from the SensESP web UI via ConfigItem below.
    auto* tank_capacity =
        new ConstantSensor<float>(0.02f, static_cast<int>(report_interval_ms / 1000), "/Sensors/Fuel Tank Capacity");
    ConfigItem(tank_capacity)
        ->set_title("Fuel Tank Capacity")
        ->set_description("Fuel tank capacity value.")
        ->set_sort_order(1000);
    tank_capacity->connect_to(new SKOutputFloat("tanks.fuel.0.capacity", "", "l"));

    const char* sk_path = "tanks.fuel.0.currentLevel";

    const float empty_value = 40.0;
    const float full_value = 0.0;

    const float multiplier = 1.0 / (full_value - empty_value);
    const float offset = -empty_value * multiplier;

    tank_level
        ->connect_to(new Linear(multiplier, offset))
        ->connect_to(new LambdaTransform<float, float>([](float v) -> float {
            if (v < 0.0f) return 0.0f;
            if (v > 1.0f) return 1.0f;
            return v;
        }))
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


    ESP_LOGI("ARDUINO", "Firmware variant: %s version: %s git: %s",
             FIRMWARE_VARIANT,
             FIRMWARE_VERSION,
             FIRMWARE_GIT_SHA);

    firmware_server.on("/firmware/status", HTTP_GET, handle_firmware_status);
    firmware_server.on("/firmware/update", HTTP_POST, handle_firmware_update);
    firmware_server.begin();

    sensesp_app->start();
    ESP_LOGI("ARDUINO", "SensESP Started Successfully!");

    if (WiFi.status() == WL_CONNECTED) {
        check_for_firmware_update();
    }
}



void loop() {
    event_loop()->tick();
    firmware_server.handleClient();

#ifdef ENABLE_MPPT
    if (mppt) {
        mppt->loop();
    }
#endif
}
