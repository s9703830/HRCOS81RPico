#include <Arduino.h>
//#include <ArduinoOTA.h> // Ardiuno OTA lib
//#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"


//Pio uses MBED OS for Raspberry Pi PICO and not FreeRTOS https://os.mbed.com/

//#include <WiFi.h>
//#include <WiFiUdp.h>
#include <WiFiEspAT.h>
//#include <WiFiClientSecure.h>

#define WIFI_RESET_PIN 2u
#ifndef WIFI_SSID
#define WIFI_SSID "MOBISOFT-2.4G"
#define WIFI_PASS "1singapoer4"
#endif

// Initialize the WiFi Ethernet client object
// WiFiEspClient espClient;
WiFiClient espClient;
//WiFiClientSecure espClient;
//PubSubClient awsMQTTClient(espClient); //  AWS MQTT Client



#include <Wire.h>
#include <SPI.h>

//define ONEWIRE_SEARCH 0 // as per https://registry.platformio.org/libraries/matmunk/DS18B20
//#define ONEWIRE_CRC 1 // CRC checks must be enabled
//#include <OneWire.h>
//#include <DallasTemperature.h>
#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_Sensor.h>
//#include <Adafruit_I2CDevice.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_LTR390.h>
//#include <Adafruit_CCS811.h>
#include <DHT.h>
#include <DHT_U.h>

// https://wiki.dfrobot.com/Gravity__Analog_pH_Sensor_Meter_Kit_V2_SKU_SEN0161-V2
// https://kevinboone.me/picoflash.html?i=1
#include "DFRobot_PH.h"
#define PH_PIN A1
DFRobot_PH ph;

// available pins per https://forum.arduino.cc/t/pi-pico-arduino-ide-pin-mapping-for-gpio-in-ide/903691
// default "Wire" object: SDA = GP4, SCL = GP5, I2C0 peripheral
// our new wire object:
#define WIRE1_SDA       (18u)  // Use GP2 as I2C1 SDA
#define WIRE1_SCL       (19u) // Use GP3 as I2C1 SCL
arduino::MbedI2C Wire1(WIRE1_SDA, WIRE1_SCL);

// default "Serial1" object: UART0, TX = GP0, RX = GP1
// our new Serial object:
//#define SERIAL1_TX      0u  // Use GP4 as UART1 TX
//#define SERIAL1_RX      1u  // Use GP5 as UART1 RX
//arduino::UART Serial1(SERIAL1_TX, SERIAL1_RX, NC, NC);

#include "DFRobot_CCS811.h"
#include "DFRobot_BME680_I2C.h"

// See guide for details on sensor wiring and usage:
// https://learn.adafruit.com/dht/overview
// https://github.com/adafruit/DHT-sensor-library
#define DHTTYPE DHT11     // DHT 11
#define DHTPIN (16u)   // GPIO16 Digital pin connected to the DHT sensor 
DHT dht(DHTPIN, DHTTYPE);
DHT_Unified dht_u(DHTPIN, DHTTYPE);

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS (22u)
#define TEMPERATURE_PRECISION 9
#define INLET_TEMPERATURE_ID "28:f4:2c:3:0:0:0:e1"
#define OUTLET_TEMPERATURE_ID "28:ae:5:3:0:0:0:b"
//DS18B20 sensorsDS18S20(ONE_WIRE_BUS);
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
//OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
//DallasTemperature sensorsDS18S20(&oneWire);
// arrays to hold device addresses
//DeviceAddress outletThermometer, inletThermometer;
#define OW_PIN  ONE_WIRE_BUS // 22u
//Set to true for parasitically powered sensors.
#define PARASITE_POWER  false
// We have 2 OneWire DS18B20 temperature sensors in use on the bus
#define CONFIG_MAX_SRCH_FILTERS 2 // Max 5 supported as per DSTherm::SUPPORTED_SLAVES_NUM
// Uncomment to set permanent, common resolution for all sensors on the bus.
// Resolution may vary from 9 to 12 bits. Higher resolution takes longer to read
//#define COMMON_RES  (DSTherm::RES_9_BIT)
#define COMMON_RES  (DSTherm::RES_12_BIT)
static Placeholder<OneWireNg_CurrentPlatform> _ow;


// available pins per https://forum.arduino.cc/t/pi-pico-arduino-ide-pin-mapping-for-gpio-in-ide/903691
// default "Wire" object: SDA = GP4, SCL = GP5, I2C0 peripheral
// our new wire object:
//#define WIRE1_SDA       (18u)  // Use GP2 as I2C1 SDA
//#define WIRE1_SCL       (19u) // Use GP3 as I2C1 SCL
//arduino::MbedI2C Wire1(WIRE1_SDA, WIRE1_SCL);



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);


// default "Serial1" object: UART0, TX = GP0, RX = GP1
// our new Serial object:
//#define SERIAL2_TX      4  // Use GP4 as UART1 TX
//#define SERIAL2_RX      5  // Use GP5 as UART1 RX
//arduino::UART Serial2(SERIAL2_TX, SERIAL2_RX, NC, NC);

// default SPI at MISO = GP16, SS = GP17, SCLK = GP18, MOSI = GP19
// SS/CS is software controlled, doesn't matter which pin
//#define SPI1_MISO 12
//#define SPI1_MOSI 11
//#define SPI1_SCLK 10
//arduino::MbedSPI SPI1(SPI1_MISO, SPI1_MOSI, SPI1_SCLK);

// I2C START Pin definition etc.
#define I2C_ADDR_TSL25911_LIGHT_SENSOR 0x29 // already defaulted to this so this define is not used
#define I2C_ADDR__UV_SENSOR 0x29
#define I2C_ADDR_OLED_SSD1306_128X64 0x3C
//Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice();
Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(); // pass in a number for the sensor identifier (for your use later)
Adafruit_LTR390 ltr390 = Adafruit_LTR390();
//Adafruit_CCS811 ccs811; // I2C addr 0x53, set in driver header file. Wire changed to Wire1 in lib to use correct I2C port on Pico
DFRobot_CCS811 CCS811(&Wire1, /*IIC_ADDRESS=*/0x5A);
//DFRobot_CCS811 CCS811;

// Use an accurate altitude to calibrate sea level air pressure
// https://randomnerdtutorials.com/esp32-bme680-sensor-arduino/
#define CALIBRATE_PRESSURE
DFRobot_BME680_I2C bme(&Wire1, 0x77);  //0x77 I2C address
float seaLevel;
// I2C END

#define DEBUG_ENABLED 1


// WiFI credentials
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;	
const char* ntpServer = "pool.ntp.org"; //"time.google.com"; //
bool internet_connected = false;
String MAC = ""; //String(WiFi.macAddress());

const char* mqtt_server = "a17cw65b0eoxkx-ats.iot.us-east-1.amazonaws.com"; // Relace with your MQTT END point
const int   mqtt_port = 8883;

time_t now;    // this is the epoch
tm timeInfo;         // the structure tm holds time information in a more convenient way
String timeEpoch =  ""; //String(timeClient.getEpochTime());
String timeEpochms =  ""; //String(timeClient.getEpochTime());



struct SensorMetrics {
  //float ccs811TempOffeset;
  uint16_t ccs811Baseline;
  uint16_t ccs811eCO2;
  uint16_t ccs811TVOC;
  float dht11Temp;
  float dht11Humidity;
  float dht11HeatIndex;
  float bme680Temp;
  float bme680Humidity;
  float bme680GasResistanceVOC;
  float bme680AirPressure; 
  float bme680Altitute;
  float bme680AltituteCalibrated; 
  float ds18b20TempInlet;
  float ds18b20TempOutlet;  
  float tsl2691IR;
  float tsl2691Full;
  float tsl2691Visible;
  float tsl2691Lux;
  float ltr390UVS;
  float ltr390ALS;
  float valuePH;
  float valuePHVoltage;
  float valueEC;
  float valueECVoltage;
} sensorMetrics; 



// Define the AWS CA Root certificate as we upload to AWS services
// https://docs.platformio.org/en/latest/platforms/espressif32.html#embedding-binary-data
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_src_certs_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_src_certs_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_src_certs_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_src_certs_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_src_certs_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_src_certs_private_pem_key_end");


void dht_sensor_init() {
  // Initialize device.
  dht.begin();
  // Set delay between sensor readings based on sensor details.
  //delayMS = sensor.min_delay / 1000;
} // end dht_sensor_init()

void dht_sensor_info_print() {
  #ifdef DEBUG_ENABLED
    Serial.println();
    Serial.println(F("----------------------- dht_sensor_info_print ---------------------------"));
    Serial.println(F("DHTxx Unified Sensor info print"));
  #endif  
  // Print temperature sensor details.
  sensor_t sensor;
  dht_u.temperature().getSensor(&sensor);

  #ifdef DEBUG_ENABLED
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
    Serial.println(F("------------------------------------"));
  #endif

  // Print humidity sensor details.
  dht_u.humidity().getSensor(&sensor);

  #ifdef DEBUG_ENABLED
    Serial.println(F("Humidity Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
    Serial.println(F("---------------------------------------------------------------"));
    Serial.println();
  #endif

} // end dht_sensor_info_print()

void dht_sensor_read_print() {
  #ifdef DEBUG_ENABLED
    Serial.println();
    Serial.println(F("----------------------- dht_sensor_read_print ---------------------------"));
    Serial.println(F("DHTxx Unified Sensor read measurements and print"));
  #endif

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float temperature = dht.readTemperature();
  float heatindex = -100.0;

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    #ifdef DEBUG_ENABLED
      Serial.println(F("Failed to read from DHT sensor!"));
    #endif
    return;
  }
  else {
    heatindex = dht.computeHeatIndex(temperature, humidity, false);
    if(isnan(heatindex)) {
      #ifdef DEBUG_ENABLED
        Serial.print(F("Failed to calculate Heat Index!"));
      #endif
    } else {
      Serial.print(F("Heat Index: "));
      Serial.print(heatindex);
      Serial.println(F("°C"));
      //if ( tb.connected() ) tb.sendTelemetryFloat("heatindex", heatindex);
      // {"ts":1451649600512, "values":{"key1":"value1", "key2":"value2"}}
      sensorMetrics.dht11Temp = temperature;
      sensorMetrics.dht11Humidity = humidity;
      sensorMetrics.dht11HeatIndex = heatindex;
      #ifdef DEBUG_ENABLED
        String telemetryJSON = "{\"ts\":"+ timeEpochms +", \"values\":{\"temperature\":\"" + sensorMetrics.dht11Temp + "\", \"humidity\":\"" + sensorMetrics.dht11Humidity + "\", \"heatindex\":\"" + sensorMetrics.dht11HeatIndex + "\"}}";
        Serial.println("Sending MQTT sensor data :> " + telemetryJSON);
      #endif
      //if ( tb.connected() ) tb.sendTelemetryJson(telemetryJSON.c_str());
      //if ( awsMQTTClient.connected() ) awsMQTTClient.publish(AWS_MQTT_TOPIC_DHT11_SENSOR, telemetryJSON.c_str());
    }
  }

  #ifdef DEBUG_ENABLED
  if (isnan(temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(temperature);
    Serial.println(F("°C"));
    //if ( tb.connected() ) tb.sendTelemetryFloat("temperature", temperature);
  }
  // Get humidity event and print its value.
    if (isnan(humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.println(F("%"));
    //if ( tb.connected() ) tb.sendTelemetryFloat("humidity", humidity);
  }
 
    Serial.println(F("---------------------------------------------------------------"));
    Serial.println();
  #endif    
} // end dht_sensor_read_print()



void displayTSL2591SensorDetails(void)
{
  Serial.println();
  Serial.println(F("----------------------- displayTSL2591SensorDetails ---------------------------"));

  sensor_t sensor;
  tsl2591.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
}


//    Configures the gain and integration time for the TSL2591
void configureTSL2591Sensor(void)
{
  Serial.println();
  Serial.println(F("------------------- configureTSL2591Sensor ---------------------"));

  if (tsl2591.begin(&Wire1)) 
  {
    Serial.println(F("Found a TSL2591 sensor"));
  } 
  else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    //while (1);
  }
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl2591.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl2591.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  // Display the gain and integration time for reference sake   
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl2591.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl2591.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println();

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
}

//    Shows how to perform a basic read on visible, full spectrum or
//    infrared light (returns raw 16-bit ADC values)
void simpleReadTSL2591Sensor(void)
{
  Serial.println();
  Serial.println(F("------------------- simpleReadTSL2591Sensor ---------------------"));
  // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 100-600 milliseconds! Uncomment whichever of the following you want to read
  uint16_t x = tsl2591.getLuminosity(TSL2591_VISIBLE);
  //uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  //uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);

  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("Luminosity: "));
  Serial.println(x, DEC);
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
}

//    Show how to read IR and Full Spectrum at once and convert to lux
void advancedReadTSL2591Sensor(void)
{
  Serial.println();
  Serial.println(F("------------------- advancedReadTSL2591Sensor ---------------------"));
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl2591.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  
  sensorMetrics.tsl2691IR = ir;
  Serial.print(F("IR: ")); Serial.print(sensorMetrics.tsl2691IR);  Serial.print(F("  "));  
  
  sensorMetrics.tsl2691Full = full;
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));  
  
  sensorMetrics.tsl2691Visible = (full - ir);
  Serial.print(F("Visible: ")); Serial.print(sensorMetrics.tsl2691Visible); Serial.print(F("  "));

    sensorMetrics.tsl2691Lux = tsl2591.calculateLux(full, ir);
  Serial.print(F("Lux: ")); Serial.println(sensorMetrics.tsl2691Lux, 6);
  
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
}

void configureLTR390sensor() {
  Serial.println();
  Serial.println(F("------------------- configureLTR390sensor ---------------------"));
  Serial.println(F("Wave or Adafruit LTR-390 UV sensor setup"));

  if ( !ltr390.begin(&Wire1) ) {
    Serial.println(F("Couldn't find LTR sensor!"));
  } else{
    Serial.println(F("Found LTR sensor!"));
  }

  ltr390.setMode(LTR390_MODE_UVS);
  if (ltr390.getMode() == LTR390_MODE_ALS) {
    Serial.println(F("In ALS mode"));
  } else {
    Serial.println(F("In UVS mode"));
  }

  ltr390.setGain(LTR390_GAIN_3);
  Serial.print(F("Gain : "));
  switch (ltr390.getGain()) {
    case LTR390_GAIN_1: Serial.println(1); break;
    case LTR390_GAIN_3: Serial.println(3); break;
    case LTR390_GAIN_6: Serial.println(6); break;
    case LTR390_GAIN_9: Serial.println(9); break;
    case LTR390_GAIN_18: Serial.println(18); break;
  }

  ltr390.setResolution(LTR390_RESOLUTION_16BIT);
  Serial.print(F("Resolution : "));
  switch (ltr390.getResolution()) {
    case LTR390_RESOLUTION_13BIT: Serial.println(13); break;
    case LTR390_RESOLUTION_16BIT: Serial.println(16); break;
    case LTR390_RESOLUTION_17BIT: Serial.println(17); break;
    case LTR390_RESOLUTION_18BIT: Serial.println(18); break;
    case LTR390_RESOLUTION_19BIT: Serial.println(19); break;
    case LTR390_RESOLUTION_20BIT: Serial.println(20); break;
  }

  ltr390.setThresholds(100, 1000);
  ltr390.configInterrupt(true, LTR390_MODE_UVS);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
  
} // configureLTR390sensor()

void readLTR390Sensor() {
  Serial.println(F("------------------- readLTR390Sensor ---------------------"));
  Serial.println(F("Wave or Adafruit LTR-390 UV sensor setup"));

  if (ltr390.newDataAvailable()) {
      Serial.println();   
      Serial.print(F("UVS data: ")); 
      sensorMetrics.ltr390UVS = ltr390.readUVS();
      Serial.print(sensorMetrics.ltr390UVS);
      Serial.print(F(" ; ")); 
      Serial.print(F("ALS data: ")); 
      sensorMetrics.ltr390ALS = ltr390.readALS();
      Serial.print(sensorMetrics.ltr390ALS);
      Serial.println();   
  }

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
} // readLTR390Sensor()

/*
void configureCSS811Sensor() {

  Serial.println(F("------------------- configureCSS811Sensor ---------------------"));
  Serial.println(F("Wave or Adafruit CSS811 Air Quality sensor setup"));  

  if(!ccs811.begin(&Wire1)){
    Serial.println(F("Failed to start sensor! Please check your wiring."));
  } else {
    Serial.println(F("Found CSS811 sensor!"));
  }
    
  // https://itbrainpower.net/downloadables/CCS811_DS000459_5-00.pdf
  //calibrate temperature sensor
  while(!ccs811.available());
  //float temp = ccs811.calculateTemperature();
  //sensorMetrics.ccs811TempOffeset = (temp - 25.0);
  //ccs811.setTempOffset(sensorMetrics.ccs811TempOffeset);
   // set the actual temperature and humidity readings from the DHT 11 sensor. Need to read DHT11 first
  ccs811.setEnvironmentalData(sensorMetrics.dht11Temp,sensorMetrics.dht11Humidity);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();

} // configureCSS811Sensor()


void readCSS811Sensor() {
  Serial.println(F("------------------- readCSS811Sensor ---------------------"));
  Serial.println(F("Wave or Adafruit CSS811 Air Quality sensor read and print values"));  

 if(ccs811.available()){
     // Update the actual current environment temperature and humidity readings from the DHT 11 sensor. Need to read DHT11 first
    ccs811.setEnvironmentalData(sensorMetrics.dht11Temp,sensorMetrics.dht11Humidity);
    float temp = ccs811.calculateTemperature();
    if(!ccs811.readData()){
      Serial.print(F("eCO2: "));
      float eCO2 = ccs811.geteCO2();
      sensorMetrics.ccs811eCO2 = eCO2;
      Serial.print(eCO2);
      
      Serial.print(F(" ppm, TVOC: "));      
      float TVOC = ccs811.getTVOC();
      sensorMetrics.ccsTVOC = TVOC;
      Serial.print(TVOC);
      
      Serial.print(F(" ppb   Temp:"));
      Serial.println(temp);

    }
    else{
      Serial.println(F("ERROR! Reading CSS811 sensor"));
    }
 }

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();    
} // end readCSS811Sensor()
*/

// https://github.com/DFRobot/DFRobot_CCS811
// https://wiki.dfrobot.com/CCS811_Air_Quality_Sensor_SKU_SEN0339
void configureCSS811Sensor() {
  Serial.println(F("------------------- configureCSS811Sensor ---------------------"));
  Serial.println(F("Wave or Adafruit CSS811 Air Quality sensor setup")); 

  /*Wait for the chip to be initialized completely, and then exit*/
  while(CCS811.begin() != 0){
      Serial.println("failed to init chip, please check if the chip connection is fine");
      delay(1000);
  }

  /**
   * Measurement parameter configuration 
   * mode:in typedef enum{
   *              eClosed,      //Idle (Measurements are disabled in this mode)
   *              eCycle_1s,    //Constant power mode, IAQ measurement every second
   *              eCycle_10s,   //Pulse heating mode IAQ measurement every 10 seconds
   *              eCycle_60s,   //Low power pulse heating mode IAQ measurement every 60 seconds
   *              eCycle_250ms  //Constant power mode, sensor measurement every 250ms 1xx: Reserved modes (For future use)
   *          }eCycle_t;
   */
  CCS811.setMeasurementMode(CCS811.eCycle_250ms);

  // set environmental data for temperarature and humidity, must read the DHT11 sensor data first before initializing this sensor
  CCS811.setInTempHum(/*temperature=*/sensorMetrics.dht11Temp,/*humidity=*/sensorMetrics.dht11Humidity);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();
} // end configureCSS811Sensor()

// https://wiki.dfrobot.com/CCS811_Air_Quality_Sensor_SKU_SEN0339
void setCSS811BaseLine() {
  Serial.println(F("------------------- setCSS811BaseLine ---------------------"));
  Serial.println(F("DFRobot CSS811 Air Quality sensor get baseline"));

    if(CCS811.checkDataReady() == true){
        /*!
         * @brief Set baseline
         * @return baseline in clear air
         */
        sensorMetrics.ccs811Baseline = CCS811.readBaseLine();
        Serial.println(CCS811.readBaseLine(), HEX);

    } else {
        Serial.println("Data is not ready!");
    }

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();    
} // end setCSS811BaseLine()

void readCSS811Sensor() {
  Serial.println(F("------------------- readCSS811Sensor ---------------------"));
  Serial.println(F("DFRobot CSS811 Air Quality sensor read and print values"));
  
  // set environmental data for temperarature and humidity, must read the DHT11 sensor data first before initializing this sensor
  // Ensure that we update the CCS811 sensor algorithm with the currently realtime measured temperature and humidity levels
  CCS811.setInTempHum(/*temperature=*/sensorMetrics.dht11Temp,/*humidity=*/sensorMetrics.dht11Humidity);

    if(CCS811.checkDataReady() == true){
        Serial.print("CO2: ");
        sensorMetrics.ccs811eCO2 = CCS811.getCO2PPM();
        Serial.print(sensorMetrics.ccs811eCO2);
        Serial.print("ppm, TVOC: ");
        sensorMetrics.ccs811TVOC = CCS811.getTVOCPPB();
        Serial.print(sensorMetrics.ccs811TVOC);
        Serial.println("ppb");
        
    } else {
        Serial.println("Data is not ready!");
    }
    /*!
     * @brief Set baseline
     * @param get from getBaseline.ino
     */
    CCS811.writeBaseLine(sensorMetrics.ccs811Baseline);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();    
} // end readCSS811Sensor()


void configureBME680()
{
  Serial.println("------------------- configureBME680 ---------------------");
  Serial.println("DFRobot BME680 Environment sensor configure");

  uint8_t rslt = 1;
  Serial.println();
  while(rslt != 0) {
    rslt = bme.begin();
    if(rslt != 0) {
      Serial.println("bme begin failure");
      delay(2000);
    }
  }
  Serial.println("bme begin successful");
  #ifdef CALIBRATE_PRESSURE
  bme.startConvert();
  delay(1000);
  bme.update();
  /*You can use an accurate altitude to calibrate sea level air pressure. 
   *And then use this calibrated sea level pressure as a reference to obtain the calibrated altitude.
   *In this case,525.0m is chendu accurate altitude.
   */
  seaLevel = bme.readSeaLevel(105.0);
  Serial.print("seaLevel :");
  Serial.println(seaLevel);
  #endif

  Serial.println("---------------------------------------------------------------");
  Serial.println();    
} // end configureBME680()

void readBME680()
{
  Serial.println("------------------------ readBME680 ---------------------------");
  Serial.println("DFRobot BME680 Environment sensor read and print values");

  bme.startConvert();
  delay(1000);
  bme.update();
  Serial.println();

  Serial.print("temperature(C) :");
  sensorMetrics.bme680Temp = bme.readTemperature() / 100;
  Serial.println(sensorMetrics.bme680Temp, 2);
  Serial.print("pressure(Pa) :");
  sensorMetrics.bme680AirPressure = bme.readPressure();
  Serial.println(sensorMetrics.bme680AirPressure);
  Serial.print("humidity(%rh) :");
  sensorMetrics.bme680Humidity = bme.readHumidity() / 1000;
  Serial.println(sensorMetrics.bme680Humidity, 2);
  Serial.print("gas resistance(ohm) :");
  sensorMetrics.bme680GasResistanceVOC = bme.readGasResistance();
  Serial.println(sensorMetrics.bme680GasResistanceVOC);
  Serial.print("altitude(m) :");
  sensorMetrics.bme680Altitute = bme.readAltitude();
  Serial.println(sensorMetrics.bme680Altitute);
  #ifdef CALIBRATE_PRESSURE
  Serial.print("calibrated altitude(m) :");
  sensorMetrics.bme680AltituteCalibrated = bme.readCalibratedAltitude(seaLevel);
  Serial.println(sensorMetrics.bme680AltituteCalibrated);
  #endif

  Serial.println("---------------------------------------------------------------");
  Serial.println();
} // end readBME680()


void printTextXY_SSD1306(int16_t x, int16_t y, String str) {
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.println(str);
} // printTextXY_SSD1306

void printSensorMetrics_SSD1306() {
  Serial.println(F("------------------- printSensorMetrics_SSD1306 ---------------------"));
  Serial.println(F("Adafruit SSD1306 display update"));    
  display.clearDisplay();
  String dht11MetricsStr = "T:" + String(sensorMetrics.dht11Temp,1) + " H:" + String(sensorMetrics.dht11Humidity,1) + " HI:" + String(sensorMetrics.dht11HeatIndex,1);
  printTextXY_SSD1306(0, 0, dht11MetricsStr);
  
  String tsl2691MetricsStr = "IR:" + String(sensorMetrics.tsl2691IR,1) + " Lx:" + String(sensorMetrics.tsl2691Lux,1) + " Vis:" + String(sensorMetrics.tsl2691Visible,1);
  printTextXY_SSD1306(0, 10, tsl2691MetricsStr);

  String ltr390MetricsStr = "UVS:" + String(sensorMetrics.ltr390UVS,1) + " ALS:" + String(sensorMetrics.ltr390ALS,1);
  printTextXY_SSD1306(0, 30, ltr390MetricsStr);

  String ccs811MetricsStr = "eCO2:" + String(sensorMetrics.ccs811eCO2) + " TVOC:" + String(sensorMetrics.ccs811TVOC); // + "   Toffset:" + String(sensorMetrics.ccs811TempOffeset,2);
  printTextXY_SSD1306(0, 41, ccs811MetricsStr);

  String ds18b20MetricsStr = "Ti:" + String(sensorMetrics.ds18b20TempInlet,2) + " To:" + String(sensorMetrics.ds18b20TempOutlet,2); 
  printTextXY_SSD1306(0, 51, ds18b20MetricsStr);

  String bme680MetricsStr = "T: " + String(sensorMetrics.bme680Temp,1) + " H: " + String(sensorMetrics.bme680Humidity,1) + " Pa: " + String(sensorMetrics.bme680AirPressure,1)
   + " Alt: " + String(sensorMetrics.bme680Altitute,1)  + " AltC: " + String(sensorMetrics.bme680AltituteCalibrated,1)  + " VOC: " + String(sensorMetrics.bme680GasResistanceVOC,1);
  printTextXY_SSD1306(0, 51, bme680MetricsStr);

  display.display();      // Show initial text
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
} // end printSensorMetrics_SSD1306()

void printSensorMetrics_Terminal() {
  Serial.println(F("------------------- printSensorMetrics_Terminal ---------------------"));

  Serial.println(F("dht11MetricsStr :"));
  String dht11MetricsStr = "T: " + String(sensorMetrics.dht11Temp,2) + " H: " + String(sensorMetrics.dht11Humidity,2) + " HI: " + String(sensorMetrics.dht11HeatIndex,2);
  Serial.println(dht11MetricsStr);
  
  Serial.println(F("bme680MetricsStr :"));
  String bme680MetricsStr = "T: " + String(sensorMetrics.bme680Temp,2) + " H: " + String(sensorMetrics.bme680Humidity,2) + " Pa: " + String(sensorMetrics.bme680AirPressure,2)
   + " Alt: " + String(sensorMetrics.bme680Altitute,2)  + " AltCalibrated: " + String(sensorMetrics.bme680AltituteCalibrated,2)  + " VOC: " + String(sensorMetrics.bme680GasResistanceVOC,2);
  Serial.println(bme680MetricsStr);

  Serial.println(F("tsl2691MetricsStr :"));
  String tsl2691MetricsStr = "IR: " + String(sensorMetrics.tsl2691IR,2) + " Lux: " + String(sensorMetrics.tsl2691Lux,2) + " Vis: " + String(sensorMetrics.tsl2691Visible,2) + " Full: " + String(sensorMetrics.tsl2691Full,2);
  Serial.println(tsl2691MetricsStr);

  Serial.println(F("ltr390MetricsStr :"));
  String ltr390MetricsStr = "UVS: " + String(sensorMetrics.ltr390UVS,2) + " ALS: " + String(sensorMetrics.ltr390ALS,2);
  Serial.println(ltr390MetricsStr);

  Serial.println(F("ltr390MetricsStr :"));
  String ccs811MetricsStr = "eCO2:" + String(sensorMetrics.ccs811eCO2) + " TVOC:" + String(sensorMetrics.ccs811TVOC); // + "   Toffset:" + String(sensorMetrics.ccs811TempOffeset,2);
  Serial.println(ccs811MetricsStr);

  Serial.println(F("ds18b20MetricsStr :"));
  String ds18b20MetricsStr = "Ti:" + String(sensorMetrics.ds18b20TempInlet,2) + " To:" + String(sensorMetrics.ds18b20TempOutlet,2); 
  Serial.println(ds18b20MetricsStr);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
} // end printSensorMetrics_SSD1306()


void configureSSD1306Display() {
  Serial.println(F("------------------- configureSSD1306Display ---------------------"));
  Serial.println(F("Adafruit SSD1306 display configure"));  

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    Serial.println(F("SSD1306 allocation success!"));
  }
  
  //display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();

  // Clear the buffer
  display.clearDisplay();

  //String helloStr = "HRCOS82 Project 10", Markus Haywood, UNISA Student No: 34495495";
  printTextXY_SSD1306(0, 0, "HRCOS82 Project 10");
  printTextXY_SSD1306(0, 10, "Markus Haywood");
  printTextXY_SSD1306(0, 20, "UNISA");
  printTextXY_SSD1306(0, 30, "Student No: 34495495");
  display.display();

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
}

// https://github.com/arduino/ArduinoCore-mbed/blob/master/libraries/Wire/Wire.h#L37
// https://community.platformio.org/t/change-pi-pico-serial-and-i2c-pins-and-use-both-i2c-ports/27902/4
void i2c_setup() {
  Wire1.begin();
} // end i2c_setup()



/* returns false if not supported */
// OneWireNg
static bool printDS18S20Id(const OneWireNg::Id& id)
{
    const char *name = DSTherm::getFamilyName(id);

    Serial.print(id[0], HEX);
    for (size_t i = 1; i < sizeof(OneWireNg::Id); i++) {
        Serial.print(':');
        Serial.print(id[i], HEX);
    }
    if (name) {
        Serial.print(" -> ");
        Serial.print(name);
    }
    Serial.println();

    return (name != NULL);
}

// OneWireNg
static void printDS18S20Scratchpad(const OneWireNg::Id &id, const DSTherm::Scratchpad& scrpd)
{
    //const uint8_t *scrpd_raw = scrpd.getRaw();

    Serial.print("  Scratchpad:");
    String idStr = String(id[0], HEX);
    for (size_t i = 1; i < sizeof(OneWireNg::Id); i++) {
        idStr += ':' + String(id[i], HEX);
    }
    Serial.print(idStr);

    Serial.print("; Th:");
    Serial.print(scrpd.getTh());

    Serial.print("; Tl:");
    Serial.print(scrpd.getTl());

    Serial.print("; Resolution:");
    Serial.print(9 + (int)(scrpd.getResolution() - DSTherm::RES_9_BIT));

    long temp = scrpd.getTemp();
    Serial.print("; Temp:");
    if (temp < 0) {
        temp = -temp;
        Serial.print('-');
    }
    Serial.print(temp / 1000);
    Serial.print('.');
    Serial.print(temp % 1000);
    Serial.print(" C");

    float tempf =  (temp / 1000.0f);
    Serial.print(" tempf : ");
    Serial.print(tempf);
    Serial.print(" C");


    if(idStr == INLET_TEMPERATURE_ID) {
      sensorMetrics.ds18b20TempInlet = tempf;
    } else 
    if(idStr == OUTLET_TEMPERATURE_ID) {
      sensorMetrics.ds18b20TempOutlet = tempf;
    }



    Serial.println();
}

void configurSensorsDS18S20() {
  Serial.println(F("------------------- configurSensorsDS18S20 ---------------------"));
  Serial.println(F("Dallas Temperature IC Control Library start"));   
  
  new (&_ow) OneWireNg_CurrentPlatform(OW_PIN, false);
  DSTherm drv(_ow);
  drv.filterSupportedSlaves();
    // Set common resolution for all sensors. Th, Tl (high/low alarm triggers) are set to 0.
  drv.writeScratchpadAll(0, 0, COMMON_RES);
  // The configuration above is stored in volatile sensors scratchpad
  // memory and will be lost after power unplug. Therefore store the
  // configuration permanently in sensors EEPROM.
  drv.copyScratchpadAll(PARASITE_POWER);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
} // end configurSensorsDS18S20()

void readSensorsDS18S20() {
  //Serial.println(F("------------------- readSensorsDS18S20 ---------------------"));
  //Serial.println(F("Adafruit readSensorsDS18S20 display configure")); 

    DSTherm drv(_ow);
    Placeholder<DSTherm::Scratchpad> _scrpd;

    /* convert temperature on all sensors connected... */
    drv.convertTempAll(DSTherm::SCAN_BUS, PARASITE_POWER);

    /* ...and read them one-by-one */
    for (const auto& id: (OneWireNg&)_ow) {
        if (printDS18S20Id(id)) {
            if (drv.readScratchpad(id, &_scrpd) == OneWireNg::EC_SUCCESS)
                printDS18S20Scratchpad(id, _scrpd);
            else
                Serial.println("  Invalid CRC!");
        }
    }

  //Serial.println(F("---------------------------------------------------------------"));
  //Serial.println();   
} // end readSensorsDS18S20()


void configureDFRobotPHv2(){
  Serial.println(F("------------------- configureDFRobotPHv2 ---------------------"));
  Serial.println(F("Adafruit DFRobot PH V2 sensor configure")); 

  pinMode(PH_PIN,INPUT);
  //adc_gpio_init(PH_PIN);
  //adc_init();
  //adc_select_input(0); // 0 is GPIO26 A0
  analogReadResolution(12); // set analouge read resolution to 12 bits, default is 10
  ph.begin();

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
} // 

float readReservoirWaterTemperature()
{
  return sensorMetrics.ds18b20TempInlet; // make sure this is set before calling readDFRobotPHv2()
}

void readDFRobotPHv2()
{
  Serial.println(F("------------------- readDFRobotPHv2 ---------------------"));
  Serial.println(F("Adafruit DFRobot PH V2 sensor read"));    

  sensorMetrics.valuePHVoltage = analogRead(PH_PIN)/1024.0*5000;  // read the voltage
  sensorMetrics.valuePH = ph.readPH(sensorMetrics.valuePHVoltage, readReservoirWaterTemperature());  // convert voltage to pH with temperature compensation
  Serial.print("temperature: ");
  Serial.print(readReservoirWaterTemperature(),1);
  Serial.print(" ^C  pH: ");
  Serial.println(sensorMetrics.valuePH, 2);
  Serial.print("phVoltage: ");
  Serial.println(sensorMetrics.valuePHVoltage, 2);
//  ph.calibration(sensorMetrics.valuePHVoltage, readReservoirWaterTemperature());           // calibration process by Serail CMD

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
}


bool connect_wifi() {
  Serial.println(F("-------------------- WiFi connect_wifi() --------------------------"));
  int connAttempts = 0;
  bool connected = false;

  pinMode(WIFI_RESET_PIN, OUTPUT); // set onboard led pin to output
  digitalWrite(WIFI_RESET_PIN, HIGH); // Low toggle resets the module, high enables

  Serial1.begin(115200);  // WiFi module ESP-M3 hang on Serial1 PINS GP0(TX) and GP1(RX)
  while (!Serial1);

  WiFi.init(Serial1, WIFI_RESET_PIN); 
  //WiFi.init(Serial1); 

  //if (WiFi.status() == WL_NO_MODULE) {
  //  Serial.println("Communication with WiFi module failed!");
    // don't continue
  //  return false;
  //}else{
    char ver[10];
    if (WiFi.firmwareVersion(ver)) {
     Serial.print("AT firmware version ");
     Serial.println(ver);
    } 
  //}

  // disconnect if connection possibly already exists, before connecting
  if(internet_connected) WiFi.disconnect();

  //espClient.setCACert((char*)aws_root_ca_pem_start);
  //espClient.setCertificate((char*)certificate_pem_crt_start);
  //espClient.setPrivateKey((char*)private_pem_key_start);

  Serial.println(F("-------------------- WiFi CONNECTING --------------------------"));
  Serial.println("\r\nConnecting to: " + String(ssid));
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED ) {
    //vTaskDelay(1000 / portTICK_PERIOD_MS);
    delay(1000);
    Serial.print(".");
    if (connAttempts > 10) { // try to connect 10 times with a 1 second delay before giving up
       connected =  false;
       Serial.println("\r\nConnecting to: " + String(ssid) + "FAILED!!");
    }
    connAttempts++;
  }
  Serial.println();
  if(WiFi.status() == WL_CONNECTED ){
    connected = true;    
    //MAC = WiFi.macAddress();
    Serial.println(F("WiFI Internet connected"));
  }
  Serial.println(F("---------------------------------------------------------------"));
  internet_connected = connected;
  return connected;
}


void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200); //  Start the serial port for terminal communication
  while (!Serial);

  Serial.println("---------------------------- SETUP -------------------------------");

  connect_wifi();

  pinMode(LED_BUILTIN, OUTPUT); // set onboard led pin to output
  
  configureSSD1306Display();

  //pinMode(DHTPIN, INPUT_PULLUP); // set in driver lib now, no deed anymore
  dht_sensor_init(); // Initilaise the DHT11 Temperature and Humitiy sensor
  dht_sensor_info_print();

  configurSensorsDS18S20(); // Configure the water reservior Inlet and Outlet temperature sensors

  i2c_setup(); // setup i2c bus
  configureCSS811Sensor(); // set up CSS811 Air Quality sensor
  setCSS811BaseLine(); // needs to bo done once every 24h for new sensor and once a mont for burned in sensor
  configureTSL2591Sensor(); // Initilaise TSL2591 Light Sensor
  #ifdef DEBUG_ENABLED
   displayTSL2591SensorDetails();
  #endif
  configureLTR390sensor(); // set up TR390 UV sensor
  configureBME680(); // set up the DFRobot BME680 environment sensor

  configureDFRobotPHv2();

  Serial.println("---------------------------------------------------------------");
  Serial.println();   
} // end setup()


void loop() {
  // put your main code here, to run repeatedly:
  
  static unsigned long timepoint = millis();
  if(millis()-timepoint>10000U){                  //time interval: 1s

      timepoint = millis();

      Serial.println("RapBerry Pi Pico saying: LED On");
//      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
//      Serial.println("RapBerry Pi Pico saying: LED Off");
      digitalWrite(LED_BUILTIN, LOW);
      //delay(5000);

//      dht_sensor_read_print();

      readSensorsDS18S20();

//      readCSS811Sensor();
//      advancedReadTSL2591Sensor();
//      readLTR390Sensor();
//      readBME680();
    
      readDFRobotPHv2();
      
//      printSensorMetrics_Terminal();
      printSensorMetrics_SSD1306();
      
  }
  ph.calibration(sensorMetrics.valuePHVoltage, readReservoirWaterTemperature());           // calibration process by Serail CMD

} // end loop()

