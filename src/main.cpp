// Boilerplate #includes:
    #include "sensesp_app_builder.h"
    #include "sensesp/signalk/signalk_output.h"

// Sensor-specific #includes:
    #include <UltrasonicA02YYUW.h>
    #include <Stream.h>

// For RepeatSensor:
    #include "sensesp/sensors/sensor.h"
    #include <HardwareSerial.h>
    #include "sensesp/transforms/moving_average.h"
    #include "sensesp/transforms/linear.h"
    #include "sensesp/transforms/integrator.h"
    #include "sensesp/system/system_status_led.h"

    #include <sensesp.h>
    #include "mppt_rs485.h"
    #include <Wire.h>
    #include <MPU6050.h>

    #include <HardwareSerial.h>

    MPU6050 mpu;
    bool mpu_ok = false;

    float pitch = 0;
    float roll = 0;

// Internal SensESP code is wrapped in a sensesp namespace
    using namespace sensesp;

// tx of the Arduino to rx of the sensor - adjust to your own board.
    const byte txPin = 7;      

// rx of the Arduino to tx of the sensor - adjust to your own board.
    const byte rxPin = 6;                               

// Pass the sensor object to the sensor constructor.
    HardwareSerial sensorSerial(1);

// Create an instance of the sensor using the SoftwareSerial object.
    UltrasonicA02YYUW sensor(sensorSerial, rxPin, txPin);

    #define MPPT_ADDRESS 0x01
    MPPT_RS485* mppt = nullptr;

    #define I2C_SDA 8
    #define I2C_SCL 9

    // Define the function that will be called every time we want
    // an updated level from the sensor. The sensor reads in mm.
    float read_level_callback () { 
        sensor.update(); 
        return sensor.getDistance(); 
    }

    // This function determines the status of the sensor and reports back. It will return 
    // 1 if the sensor is getting a reading and 0 if it is not. 
    // Refer to A02YYUW documentation for more information on the sensor status values, timeouts etc.
    int read_sensor_status () { 
        return sensor.getDistance() > 0 ? 1 : 0;
    }
 
    void setup() {
        pinMode(rxPin, INPUT);
        pinMode(txPin, OUTPUT);
        sensorSerial.begin(9600, SERIAL_8N1, rxPin, txPin);
        sensor.begin();
        delay(3000); 
        // 1. MUST BE FIRST: Initialize SensESP v3 Logging
        SetupLogging(); 

        // 2. S3 Zero needs a moment for the USB-CDC and Flash to settle
        delay(2000); 
        ESP_LOGI("ARDUINO", "--- BOOT START ---");

        // gyrometer setup
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

        // 1. Create a builder
        auto builder = new SensESPAppBuilder();

        // 2. Manually create a SystemStatusLed object on a SAFE pin (0)
        // We use a shared_ptr because that's what the builder expects in v3
        auto safe_led = std::make_shared<SystemStatusLed>(0);

        // 3. Hand the safe LED to the builder. This prevents the "Pin 97" ghost.
        sensesp_app = builder
            ->set_hostname("SensESP")
            ->set_system_status_led(safe_led) 
            ->get_app();

        ESP_LOGI("ARDUINO", "App object created successfully");    

        auto* pitch_sensor = new RepeatSensor<float>(1500, []() {

            int16_t ax, ay, az;
            if (!mpu_ok) return 0.0f;

            mpu.getAcceleration(&ax, &ay, &az);

            float axf = ax / 16384.0;
            float ayf = ay / 16384.0;
            float azf = az / 16384.0;

            return atan2(-axf, sqrt(ayf * ayf + azf * azf));
        });

        auto* roll_sensor = new RepeatSensor<float>(1500, []() {
            int16_t ax, ay, az;
            if (!mpu_ok) return 0.0f;
            mpu.getAcceleration(&ax, &ay, &az);

            float axf = ax / 16384.0;
            float ayf = ay / 16384.0;
            float azf = az / 16384.0;

            float roll = atan2(ayf, azf);
            return roll;
        });

        // Read the sensor every 10 seconds
          unsigned int read_interval = 10000;

        // Create a RepeatSensor with float output that reads the fluid level
        // using the read_level_callback function defined above.
          auto* tank_level =
            new RepeatSensor<float>(read_interval, read_level_callback);

        // Create a RepeatSensor with float output that provides the status of the sensor as a 1 or 0. 
        // If 0 then check connections of the sensor to the arduino and to the underside of the container.
        auto* sensor_status =
            new RepeatSensor<int>(read_interval, read_sensor_status);

        
        // To make the 1s and 0s of the sensor more understandable use a Lambda Transform which takes the 
        // Integer value of sensor status and converts to true or false.
          auto int_to_bool_function = [](int sensor_status) ->bool {
            if (sensor_status == 1) {
              return true;
            }
            else { // read_sensor_status == 0
              return false;
            }
            };
          auto int_to_bool_transform = new LambdaTransform<int, bool>(int_to_bool_function);

        // Send the transformed sensor status to the Signal K server as a Bool.
          sensor_status 
                        ->connect_to(int_to_bool_transform)
                        ->connect_to(new SKOutputBool("tanks.fuel.currentLevel.sensorStatus"));

      // Set the Signal K Path for the output of the sensor.   
          const char* sk_path = "tanks.fuel.currentLevel";

      // Taking the empty level of the tank as 0 and the full level as 1000 mm, we can 
      // calculate the level of the tank as a percentage. Adjust 'full_value' to the
      // maximum height of the tank in question in mm. 
      // Full details of the method used below can be found here - 
      // https://signalk.org/SensESP/pages/tutorials/tank_level/#example-3-a-sensor-that-outputs-something-other-than-0-when-the-tank-is-empty  
          
          const float empty_value = 40.0;
          const float full_value = 0.0;

          const float multiplier = 1.0 / (full_value - empty_value);  // -0.025
          const float offset = -empty_value * multiplier;             // 1.0

      // Set the configuration paths for the Linear and MovingAverage transforms
      // that will be used to process the sensor data. This makes these values available
      // for run time configuration in the SensESP web interface.
          const char *ultrasonic_in_config_path = "tanks.fuel.currentLevel.ultrasonicIn";
          const char *linear_config_path = "tanks.fuel.currentLevel.linear";
          const char *ultrasonic_ave_samples = "tanks.fuel.currentLevel.samples";

      // Now, you add Linear to your main.cpp with your calculated multiplier and offset values:
          float scale = 1;

      // Send the fluid level of the tank to the Signal K server as a Float
          tank_level  
            ->connect_to(new Linear(multiplier, offset))   // IMPORTANT
            ->connect_to(new MovingAverage(10))
            ->connect_to(new SKOutputFloat(sk_path));

        mppt = new MPPT_RS485(MPPT_ADDRESS, 30000);  // 30 second polling
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

        pitch_sensor->connect_to(
            new SKOutputFloat("navigation.attitude.pitch"));

        roll_sensor->connect_to(
            new SKOutputFloat("navigation.attitude.roll"));

        sensesp_app->start();

        ESP_LOGI("ARDUINO", "SensESP Started Successfully!");
    }

    void update_mpu() {
        int16_t ax, ay, az;
        mpu.getAcceleration(&ax, &ay, &az);

        float axf = ax / 16384.0;
        float ayf = ay / 16384.0;
        float azf = az / 16384.0;

        roll  = atan2(ayf, azf) * 180.0 / PI;
        pitch = atan2(-axf, sqrt(ayf * ayf + azf * azf)) * 180.0 / PI;
    }

    void loop() {
        event_loop()->tick();
        if (mppt) {
            mppt->loop();
        }
        // sensesp::SensESPBaseApp::get_event_loop()->tick();
    }
