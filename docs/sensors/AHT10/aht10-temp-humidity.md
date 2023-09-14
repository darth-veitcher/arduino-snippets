# AHT10 Temperature and Humidity Sensor

This device can be found [AHT10 High Precision Digital Temperature and Humidity Sensor Measurement Module I2C Communication Replace DHT11 SHT20 AM2302](https://www.aliexpress.com/item/4000125110813.html) and at time of purchase cost £0.56 per item.

![AHT10](assets/H893db9f0c0614960983a1868b606b8b6l.jpg){ width="800" }

??? info "Breadboard Sketch"

    ![ESP8266 and AHT10 Circuit](assets/AHT%20Sketch.svg){ width="800" }

???+ tip "Arduino Code"

    The below code snippet will read the temperature and humidity from the sensor every 10 seconds and print it to serial.

    You'll need to have the following libraries installed:

    ```ini
    [env:nodemcuv2]
    platform = espressif8266
    board = nodemcuv2
    framework = arduino
    monitor_speed = 115200
    lib_deps =
        adafruit/Adafruit AHTX0@^2.0.5
    ```

    ```c++
    #include <Arduino.h>
    #define LED_BUILTIN D4  // Pin D4 or GPIO2 work. We could use LED_BUILTIN for other platforms. See https://arduino.stackexchange.com/a/38979
    #include <Adafruit_AHTX0.h>

    int setBuiltInDiode(int state);

    // intialise the DHT sensor and global variables to update in loop
    ulong last_updated = 0;
    const long update_interval = 10000;  // 10secs

    Adafruit_AHTX0 aht10;
    sensors_event_t humidity, temp;


    void setup() {
    // open the serial port at 115200 bps:
    Serial.begin(115200);
    // initialize LED digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    // start the sensor
    aht10.begin();
    }

    void loop() {
    Serial.println("Arduino has been running for " + String(time(NULL)) + " secs");

    unsigned long currentMillis = millis();
    if (currentMillis - last_updated >= update_interval) {
        if (uint(aht10.getStatus()) == 255) {
        Serial.println("ERROR: Unable to connect to AHT sesor. Device offline.");
        }
        else {
        last_updated = currentMillis;
        aht10.getEvent(&humidity, &temp);
        Serial.println("Temp: " + String(temp.temperature) + " Humidity: " + String(humidity.relative_humidity));
        }
        Serial.println("Sensor Status: " + String(aht10.getStatus()));
    }

    // arithmatically flip between 1/0
    setBuiltInDiode(1 - digitalRead(LED_BUILTIN));

    delay(1000);
    }

    int setBuiltInDiode(int state) {
    digitalWrite(LED_BUILTIN, state);
    return digitalRead(LED_BUILTIN);
    }
    ```
