//Pio uses MBED OS for Raspberry Pi PICO and not FreeRTOS https://os.mbed.com/

#include <Arduino.h>
#include <Scheduler.h>
#include <ArduinoJson.h>

#include "hardware/gpio.h"
#include "hardware/adc.h"

#include <time.h>
#include <TimeLib.h>

#include <Wire.h>
#include <SPI.h>

#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_LTR390.h>
#include <Adafruit_AS7341.h>
#include <DHT.h>
#include <DHT_U.h>

#include "DFRobot_PH.h"
#define PH_PIN A1
DFRobot_PH ph;

#include "DFRobot_EC.h"
#define EC_PIN A0
DFRobot_EC ec;

// Pins redefined in C:\Users\marku\.platformio\packages\framework-arduino-mbed\variants\RASPBERRY_PI_PICO\pins_arduino.h
// This is no longer needed
/*
#define WIRE_HOWMANY 2
#define WIRE1_SDA    (18u) // Use GP18 as I2C1 SDA
#define WIRE1_SCL    (19u) // Use GP19 as I2C1 SCL
arduino::MbedI2C Wire1(WIRE1_SDA, WIRE1_SCL);

//Use Wire0 for the Doser Motor controller PCA9685(I2C 16ch PWM) TB6612FNG(Motor Driver)
#define WIRE0_SDA       (20u) // Use GP20 as I2C0 SDA
#define WIRE0_SCL       (21u) // Use GP21 as I2C0 SCL
arduino::MbedI2C Wire0(WIRE0_SDA, WIRE0_SCL);
MbedI2C myi2c(p20,p21);

// default "Serial1" object: UART0, TX = GP0, RX = GP1
// our new Serial object:
#define SERIAL1_TX      0u  // Use GP4 as UART1 TX
#define SERIAL1_RX      1u  // Use GP5 as UART1 RX
arduino::UART Serial1(SERIAL1_TX, SERIAL1_RX, NC, NC);

// default "Serial1" object: UART0, TX = GP0, RX = GP1
// our new Serial object:
#define SERIAL2_TX      4  // Use GP4 as UART1 TX
#define SERIAL2_RX      5  // Use GP5 as UART1 RX
arduino::UART Serial2(SERIAL2_TX, SERIAL2_RX, NC, NC);

// default SPI at MISO = GP16, SS = GP17, SCLK = GP18, MOSI = GP19
// SS/CS is software controlled, doesn't matter which pin
#define SPI1_MISO 12
#define SPI1_MOSI 11
#define SPI1_SCLK 10
arduino::MbedSPI SPI1(SPI1_MISO, SPI1_MOSI, SPI1_SCLK);
*/


#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


#define AUTO_ADJUST_PH_LEVEL 6.0 
#define AUTO_ADJUST_EC_LEVEL 1.55 
#define AUTO_ADJUST_PH_INTERVAL 600000U // Check and adjust PH 10 minutes 600000U
#define AUTO_ADJUST_BROWN_INTERVAL 600000U // Check and adjust Brown 10 minutes 600000U
#define AUTO_ADJUST_GREEN_INTERVAL 600000U // Check and adjust Green 10 minutes 600000U
#define AUTO_ADJUST_PINK_INTERVAL 600000U // Check and adjust Pink 10 minutes 600000U
#define MOTOR_SHIELD_ADDRESS 0x40
Adafruit_MotorShield AFMS = Adafruit_MotorShield(MOTOR_SHIELD_ADDRESS);
Adafruit_DCMotor *motorPH = AFMS.getMotor(1);
Adafruit_DCMotor *motorBrown = AFMS.getMotor(2);
 Adafruit_DCMotor *motorGreen = AFMS.getMotor(3);
Adafruit_DCMotor *motorPink = AFMS.getMotor(4);

struct DosingMotorStates {
  uint8_t motorPHSpeed;
  uint8_t motorPHDirection;
  uint16_t motorPHRunningTime; // time to run before stop in seconds
  unsigned long motorPHStartTime;  // the time the motor started in millis
  unsigned long motorPHAdjustSleepTime;  // the time the motor started in millis
  uint8_t motorPHAutoDosingStatus; // PH Auto Dosing 1:on 0:off

  uint8_t motorBrownSpeed;
  uint8_t motorBrownDirection;
  uint16_t motorBrownRunningTime; // time to run before stop in seconds
  unsigned long motorBrownStartTime;  // the time the motor started in millis
  unsigned long motorBrownAdjustSleepTime;  // the time the motor started in millis
  uint8_t motorBrownAutoDosingStatus; // Brown Auto Dosing 1:on 0:off

  uint8_t motorGreenSpeed;
  uint8_t motorGreenDirection;
  uint16_t motorGreenRunningTime; // time to run before stop in seconds
  unsigned long motorGreenStartTime;  // the time the motor started in millis
  unsigned long motorGreenAdjustSleepTime;  // the time the motor started in millis
  uint8_t motorGreenAutoDosingStatus; // Green Auto Dosing 1:on 0:off

  uint8_t motorPinkSpeed;
  uint8_t motorPinkDirection;
  uint16_t motorPinkRunningTime; // time to run before stop in seconds
  unsigned long motorPinkStartTime;  // the time the motor started in millis
  unsigned long motorPinkAdjustSleepTime;  // the time the motor started in millis
  uint8_t motorPinkAutoDosingStatus; // Pink Auto Dosing 1:on 0:off

} motorStates;


#include <pas-co2-ino.hpp>
#include "DFRobot_CCS811.h"
#include "DFRobot_BME680_I2C.h"
#include "DFRobot_OxygenSensor.h"

/*
 * i2c slave Address, The default is ADDRESS_3.
 * ADDRESS_0   0x70  i2c device address.
 * ADDRESS_1   0x71
 * ADDRESS_2   0x72
 * ADDRESS_3   0x73
 */
#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  10             // collect number, the collection range is 1-100.
DFRobot_OxygenSensor oxygen;


// RelayBoard PINS
#define PIN_RELAY_1 (2u)
#define PIN_RELAY_2 (3u)
#define PIN_RELAY_3 (4u)
#define PIN_RELAY_4 (5u)


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
//#define INLET_TEMPERATURE_ID "28:f4:2c:3:0:0:0:e1"
//#define OUTLET_TEMPERATURE_ID "28:ae:5:3:0:0:0:b"
#define INLET_TEMPERATURE_ID "28:3e:db:3:0:0:0:46"
#define OUTLET_TEMPERATURE_ID "28:5d:1f:3:0:0:0:e1"

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


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);


// I2C START Pin definition etc.
#define I2C_ADDR_TSL25911_LIGHT_SENSOR 0x29 // already defaulted to this so this define is not used
#define I2C_ADDR__UV_SENSOR 0x29
#define I2C_ADDR_OLED_SSD1306_128X64 0x3C
Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(); // pass in a number for the sensor identifier (for your use later)
Adafruit_LTR390 ltr390 = Adafruit_LTR390();
Adafruit_AS7341 as7341 = Adafruit_AS7341();
//Adafruit_CCS811 ccs811; // I2C addr 0x53, set in driver header file. Wire changed to Wire1 in lib to use correct I2C port on Pico
DFRobot_CCS811 CCS811(&Wire1, /*IIC_ADDRESS=*/0x5A);
//DFRobot_CCS811 CCS811;

// Infineon XENSIV PAS2 https://www.infineon.com/cms/en/product/evaluation-boards/eval_pasco2_miniboard/
#define PERIODIC_MEAS_INTERVAL_IN_SECONDS  10 
#define PRESSURE_REFERENCE  1023
PASCO2Ino co2XENSIV(&Wire1);
uint8_t prodIdXENSIV, revIdXENSIV;
Error_t errXENSIV;


// Use an accurate altitude to calibrate sea level air pressure
// https://randomnerdtutorials.com/esp32-bme680-sensor-arduino/
#define CALIBRATE_PRESSURE
DFRobot_BME680_I2C bme(&Wire1, 0x77);  //0x77 I2C address
float seaLevel;
// I2C END

#define DEBUG_ENABLED 1
#define MQTT_DATA_SEND_INTERVAL 300000U // send sensor data every 10 minutes 600000U
#define SENSORS_READ_INTERVAL   30000U // read sensor data every 30 seconds


//time_t now;    // this is the epoch
tm timeInfo;         // the structure tm holds time information in a more convenient way
String timeEpoch =  ""; //String(timeClient.getEpochTime());
String timeEpochms =  ""; //String(timeClient.getEpochTime());
const int8_t TIME_ZONE = 2; // UTC + 2


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
  uint16_t as7341F1_415nm;
  uint16_t as7341F2_445nm;
  uint16_t as7341F3_480nm;
  uint16_t as7341F4_515nm;
  uint16_t as7341F5_555nm;
  uint16_t as7341F6_590nm;
  uint16_t as7341F7_630nm;
  uint16_t as7341F8_680nm;
  uint16_t as7341_clear;
  uint16_t as7341_NIR;
  float valuePH;
  float valuePHVoltage;
  int valuePHVoltageUINT16;
  float valueEC;
  float valueECVoltage;
  int valueECVoltageUINT16;
  int16_t xensivpas2CO2ppm;
  float oxygenLevel;
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
      sensorMetrics.dht11Temp = temperature;
      sensorMetrics.dht11Humidity = humidity;
      sensorMetrics.dht11HeatIndex = heatindex;
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

  delay(10000);
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
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl2591.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)
 
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

//  Show how to read IR and Full Spectrum at once and convert to lux
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

// Configure the TR390 UV Sensor
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

// Read the TR390 Sensor metrics
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


void configureAS7341sensor() {
  Serial.println(F("------------------- configureAS7341sensor ---------------------"));
  Serial.println(F("Wave or Adafruit AS7341 sensor setup"));

  if (!as7341.begin(AS7341_I2CADDR_DEFAULT, &Wire1, 0)){
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }

  Serial.println("Setup Atime, ASTEP, Gain");
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
}

void readAS7341sensor() {
  Serial.println(F("------------------- readAS7341sensor ---------------------"));
  Serial.println(F("Wave or Adafruit AS7341 sensor setup"));

  uint16_t readings[12];

  if (!as7341.readAllChannels(readings)){
    Serial.println("Error reading all channels!");
    return;
  }

  Serial.print("ADC0/F1 415nm/Violet : ");
  sensorMetrics.as7341F1_415nm = readings[0];
  Serial.println(readings[0]);

  Serial.print("ADC1/F2 445nm/Indigo : ");
  sensorMetrics.as7341F2_445nm = readings[1];
  Serial.println(readings[1]);

  Serial.print("ADC2/F3 480nm/Blue : ");
  sensorMetrics.as7341F3_480nm = readings[2];
  Serial.println(readings[2]);

  Serial.print("ADC3/F4 515nm/Cyan : ");
  sensorMetrics.as7341F4_515nm = readings[3];
  Serial.println(readings[3]);

  Serial.print("ADC0/F5 555nm/Green : ");
  sensorMetrics.as7341F5_555nm = readings[6];
  Serial.println(readings[6]);

  Serial.print("ADC1/F6 590nm/Yellow : ");
  sensorMetrics.as7341F6_590nm = readings[7];
  Serial.println(readings[7]);

  Serial.print("ADC2/F7 630nm/Orange : ");
  sensorMetrics.as7341F7_630nm = readings[8];
  Serial.println(readings[8]);

  Serial.print("ADC3/F8 680nm/Red : ");
  sensorMetrics.as7341F8_680nm = readings[9];
  Serial.println(readings[9]);

  Serial.print("ADC4/Clear    : ");
  sensorMetrics.as7341_clear = readings[10];
  Serial.println(readings[10]);

  Serial.print("ADC5/NIR      : ");
  sensorMetrics.as7341_NIR = readings[11];
  Serial.println(readings[11]);

  Serial.println();

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
}




// https://github.com/DFRobot/DFRobot_CCS811
// https://wiki.dfrobot.com/CCS811_Air_Quality_Sensor_SKU_SEN0339
// Configure the CSS811Sensor
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

// Coonfigure the Infineon XENSIV PAS2 CO2 sensor
void configureXENSIVPAS2() {
  Serial.println("------------------- configureXENSIVPAS2 ---------------------");
  Serial.println("Infineon XENSIV PAS2 CO2 sensor configure");

  /* Initialize the sensor */
  errXENSIV = co2XENSIV.begin();
  if(XENSIV_PASCO2_OK != errXENSIV)
  {
    Serial.print("initialization error: ");
    Serial.println(errXENSIV);
  }

  errXENSIV = co2XENSIV.getDeviceID(prodIdXENSIV, revIdXENSIV);
  if(XENSIV_PASCO2_OK != errXENSIV)
  {
    Serial.print("error: ");
    Serial.println(errXENSIV);
  }

  Serial.print("product id  : ");
  Serial.println(prodIdXENSIV);
  Serial.print("revision id : ");
  Serial.println(revIdXENSIV);

  /* We can set the reference pressure before starting 
    * the measure 
    */
  errXENSIV = co2XENSIV.setPressRef(PRESSURE_REFERENCE);
  if(XENSIV_PASCO2_OK != errXENSIV)
  {
    Serial.print("pressure reference error: ");
    Serial.println(errXENSIV);
  }

  /*
    * Configure the sensor to measureme periodically 
    * every 10 seconds
    */
  errXENSIV = co2XENSIV.startMeasure(PERIODIC_MEAS_INTERVAL_IN_SECONDS);
  if(XENSIV_PASCO2_OK != errXENSIV)
  {
    Serial.print("start measure error: ");
    Serial.println(errXENSIV);
  }

  Serial.println("---------------------------------------------------------------");
  Serial.println();  
}

// Read the Infineon XENSIV PAS2 CO2 sensor
void readXENSIVPAS2() {
  Serial.println("------------------- readXENSIVPAS2 ---------------------");
  Serial.println("Infineon XENSIV PAS2 CO2 sensor read");

  errXENSIV = co2XENSIV.getCO2(sensorMetrics.xensivpas2CO2ppm);
  if(XENSIV_PASCO2_OK != errXENSIV) {
      /* Retry in case of timing synch mismatch */
      if(XENSIV_PASCO2_ERR_COMM == errXENSIV)
      {
        delay(600);
        errXENSIV = co2XENSIV.getCO2(sensorMetrics.xensivpas2CO2ppm);
        if(XENSIV_PASCO2_OK != errXENSIV)          
        {
          Serial.print("get co2 error: ");
          Serial.println(errXENSIV);
        }
      }
    }

    Serial.print("co2 ppm value : ");
    Serial.println(sensorMetrics.xensivpas2CO2ppm);

    /*
     * Set the pressure reference (i.e. a pressure sensor),
     * we could compensate again by setting the new reference. 
     * Here we just keep the initial value.
     */
    if(sensorMetrics.bme680AirPressure > 0 ) { // if the bme680 pressure reading is ready use that, otherwise use default reference
      errXENSIV = co2XENSIV.setPressRef(sensorMetrics.bme680AirPressure * 0.01); // convert Pa to hPa
    } else {
      errXENSIV = co2XENSIV.setPressRef(PRESSURE_REFERENCE);
    }
    if(XENSIV_PASCO2_OK != errXENSIV)
    {
      Serial.print("pressure reference error: ");
      Serial.println(errXENSIV);
    }

  Serial.println("---------------------------------------------------------------");
  Serial.println();  
}


void configureDFRobotOxygenSensor() {
  Serial.println("------------------- configureDFRobotOxygenSensor ---------------------");
  Serial.println("DFRobot Oxygen Sensor SEN0322 configure");
  while(!oxygen.begin(Oxygen_IICAddress)){
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("I2c connect success !");
  Serial.println("---------------------------------------------------------------");
  Serial.println();  
}

void readDFRobotOxygenSensor() {
  Serial.println("------------------- readDFRobotOxygenSensor ---------------------");
  Serial.println("DFRobot Oxygen Sensor SEN0322 read");  
  float oxygenData = oxygen.getOxygenData(COLLECT_NUMBER);
  sensorMetrics.oxygenLevel = oxygenData;
  Serial.print(" oxygen concentration is ");
  Serial.print(oxygenData);
  Serial.println(" %vol");
  Serial.println("---------------------------------------------------------------");
  Serial.println();  
}


// Configure the BME680 Environment sensor
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

// Read the BME680 Environment sensor
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

// Write text to SSD1306 OLED display at position X,Y
void printTextXY_SSD1306(int16_t x, int16_t y, String str) {
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.println(str);
} // printTextXY_SSD1306


u_int8_t printLCDno = 0; // not all text fit on the display, we will print a subset of values every main loop cycle

// Write Sensor metrics to SSD1306 OLED display a sensor category at a Time to make it fit into the screen
void printSensorMetrics_SSD1306() {
  Serial.println(F("------------------- printSensorMetrics_SSD1306 ---------------------"));
  Serial.println(F("Adafruit SSD1306 display update"));    

  if (printLCDno >= 3) {  printLCDno = 0; } else printLCDno++;

  display.clearDisplay();

  switch (printLCDno)
  {
    case 0: {
            printTextXY_SSD1306(0, 0, "HRCOS82 Project 10");
            printTextXY_SSD1306(0, 10, "Markus Haywood");
            printTextXY_SSD1306(0, 20, "UNISA");
            printTextXY_SSD1306(0, 30, "Student No: 34495495");
            display.display();
      break;
    }
    case 1: { // Print the Ligth Sensor Metrics
            String lightMetricsLabelStr = "Light Metrics:"; 
            printTextXY_SSD1306(0, 0, lightMetricsLabelStr);

            String tsl2691MetricsStr1 = "IR:" + String(sensorMetrics.tsl2691IR,1) + " Lx:" + String(sensorMetrics.tsl2691Lux,1);
            printTextXY_SSD1306(0, 10, tsl2691MetricsStr1);

            String tsl2691MetricsStr2 = "Vis:" + String(sensorMetrics.tsl2691Visible,1);
            printTextXY_SSD1306(0, 20, tsl2691MetricsStr2);

            String ltr390MetricsStr = "UVS:" + String(sensorMetrics.ltr390UVS,1) + " ALS:" + String(sensorMetrics.ltr390ALS,1);
            printTextXY_SSD1306(0, 30, ltr390MetricsStr);
      break;
    }

    case 2 : { // Print the Air Sensor metrics
            String airMetricsLabelStr = "Air Metrics:"; 
            printTextXY_SSD1306(0, 0, airMetricsLabelStr);

            //String dht11MetricsStr = "T:" + String(sensorMetrics.dht11Temp,1) + " H:" + String(sensorMetrics.dht11Humidity,1) + " HI:" + String(sensorMetrics.dht11HeatIndex,1);
            //printTextXY_SSD1306(0, 10, dht11MetricsStr);

            String bme680MetricsStr1 = "T: " + String(sensorMetrics.bme680Temp,1) + " H: " + String(sensorMetrics.bme680Humidity,1);
            printTextXY_SSD1306(0, 10, bme680MetricsStr1);

            String bme680MetricsStr2 = "Alt: " + String(sensorMetrics.bme680Altitute,1)  + " AltC: " + String(sensorMetrics.bme680AltituteCalibrated,1);
            printTextXY_SSD1306(0, 20, bme680MetricsStr2);

            String bme680MetricsStr3 =  "hPa: " + String(sensorMetrics.bme680AirPressure * 0.01 ,1) + " VOC: " + String(sensorMetrics.bme680GasResistanceVOC,1);
            printTextXY_SSD1306(0, 30, bme680MetricsStr3);

            String ccs811MetricsStr = "eCO2:" + String(sensorMetrics.ccs811eCO2) + " TVOC:" + String(sensorMetrics.ccs811TVOC); // + "   Toffset:" + String(sensorMetrics.ccs811TempOffeset,2);
            printTextXY_SSD1306(0, 40, ccs811MetricsStr);

            String xensivpas2 = "CO2 ppm: " + String(sensorMetrics.xensivpas2CO2ppm,1);
            printTextXY_SSD1306(0, 50, xensivpas2);

      break;
   }

   case 3 : { // Print the Water Nutrient flued metrics
            String waterMetricsLabelStr = "Water Metrics:"; 
            printTextXY_SSD1306(0, 0, waterMetricsLabelStr);

            String ds18b20MetricsStr = "Tin:" + String(sensorMetrics.ds18b20TempInlet,2) + " Tout:" + String(sensorMetrics.ds18b20TempOutlet,2); 
            printTextXY_SSD1306(0, 10, ds18b20MetricsStr);

            String phMetricsStr = "PH:" + String(sensorMetrics.valuePH,2) + " PH Volt:" + String(sensorMetrics.valuePHVoltage,2); 
            printTextXY_SSD1306(0, 20, phMetricsStr);            

            String ecMetricsStr = "EC:" + String(sensorMetrics.valueEC,2) + " EC Volt:" + String(sensorMetrics.valueECVoltage,2); 
            printTextXY_SSD1306(0, 30, ecMetricsStr);            
     break;
   }

  } // end switch (printLCDno)

  display.display();      // Show initial text

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
} // end printSensorMetrics_SSD1306()


// Write Sensort Metrics to the serail terminal of the Raspberry Pi Pico
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
  String ccs811MetricsStr = "eCO2:" + String(sensorMetrics.ccs811eCO2) + " TVOC:" + String(sensorMetrics.ccs811TVOC) + " ccs811Baseline:" + String(sensorMetrics.ccs811Baseline,2);
  Serial.println(ccs811MetricsStr);

  Serial.println(F("ds18b20MetricsStr :"));
  String ds18b20MetricsStr = "Ti:" + String(sensorMetrics.ds18b20TempInlet,2) + " To:" + String(sensorMetrics.ds18b20TempOutlet,2); 
  Serial.println(ds18b20MetricsStr);

  Serial.println(F("ds18b20MetricsStr :"));
  String waterPHMetricsStr = "Ti:" + String(sensorMetrics.valuePH,2) + " To:" + String(sensorMetrics.valuePHVoltage,2); 
  Serial.println(waterPHMetricsStr);

  Serial.println(F("ds18b20MetricsStr :"));
  String waterECMetricsStr = "Ti:" + String(sensorMetrics.valueEC,2) + " To:" + String(sensorMetrics.valueECVoltage,2); 
  Serial.println(waterECMetricsStr);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
} // end printSensorMetrics_SSD1306()

// Convert Sensor Merics to JSON string
String getSensorMetrics_JSON() {

  String telemetryJSON = "{\"ts\":"+ timeEpochms 
  +", \"values\":{\"dht11Temp\":" + sensorMetrics.dht11Temp 
  + ", \"dht11Humidity\":" + sensorMetrics.dht11Humidity 
  + ", \"dht11HeatIndex\":" + sensorMetrics.dht11HeatIndex 
  + ", \"ccs811Baseline\":" + sensorMetrics.ccs811Baseline 
  + ", \"ccs811eCO2\":" + sensorMetrics.ccs811eCO2 
  + ", \"ccs811TVOC\":" + sensorMetrics.ccs811TVOC 
  + ", \"bme680Temp\":" + sensorMetrics.bme680Temp 
  + ", \"bme680Humidity\":" + sensorMetrics.bme680Humidity 
  + ", \"bme680GasResistanceVOC\":" + sensorMetrics.bme680GasResistanceVOC 
  + ", \"bme680AirPressure\":" + sensorMetrics.bme680AirPressure 
  + ", \"bme680Altitute\":" + sensorMetrics.bme680Altitute 
  + ", \"bme680AltituteCalibrated\":" + sensorMetrics.bme680AltituteCalibrated 
  + ", \"xensivpas2CO2ppm\":" + sensorMetrics.xensivpas2CO2ppm  
  + ", \"oxygenLevel\":" + sensorMetrics.oxygenLevel  
  + ", \"ds18b20TempInlet\":" + sensorMetrics.ds18b20TempInlet 
  + ", \"ds18b20TempOutlet\":" + sensorMetrics.ds18b20TempOutlet 
  + ", \"tsl2691IR\":" + sensorMetrics.tsl2691IR
  + ", \"tsl2691Full\":" + sensorMetrics.tsl2691Full
  + ", \"tsl2691Visible\":" + sensorMetrics.tsl2691Visible
  + ", \"tsl2691Lux\":" + sensorMetrics.tsl2691Lux
  + ", \"ltr390UVS\":" + sensorMetrics.ltr390UVS
  + ", \"ltr390ALS\":" + sensorMetrics.ltr390ALS
  + ", \"as7341F1_415nm\":" + sensorMetrics.as7341F1_415nm
  + ", \"as7341F2_445nm\":" + sensorMetrics.as7341F2_445nm
  + ", \"as7341F3_480nm\":" + sensorMetrics.as7341F3_480nm
  + ", \"as7341F4_515nm\":" + sensorMetrics.as7341F4_515nm
  + ", \"as7341F5_555nm\":" + sensorMetrics.as7341F5_555nm
  + ", \"as7341F6_590nm\":" + sensorMetrics.as7341F6_590nm
  + ", \"as7341F7_630nm\":" + sensorMetrics.as7341F7_630nm
  + ", \"as7341F8_680nm\":" + sensorMetrics.as7341F8_680nm
  + ", \"as7341_clear\":" + sensorMetrics.as7341_clear
  + ", \"as7341_NIR\":" + sensorMetrics.as7341_NIR
  + ", \"valuePH\":" + sensorMetrics.valuePH
  + ", \"valuePHVoltage\":" + sensorMetrics.valuePHVoltage
  + ", \"valueEC\":" + sensorMetrics.valueEC
  + ", \"valueECVoltage\":" + sensorMetrics.valueECVoltage
  + ", \"relay1Status\":" + digitalRead(PIN_RELAY_1)
  + ", \"relay2Status\":" + digitalRead(PIN_RELAY_2)
  + ", \"relay3Status\":" + digitalRead(PIN_RELAY_3)
  + ", \"relay4Status\":" + digitalRead(PIN_RELAY_4)
  + ", \"motorPHAdjustSleepTime\":" + motorStates.motorPHAdjustSleepTime
  + ", \"motorPHAutoDosingStatus\":" + motorStates.motorPHAutoDosingStatus
  + ", \"motorPHRunningTime\":" + motorStates.motorPHRunningTime
  + ", \"motorPHSpeed\":" + motorStates.motorPHSpeed
  + ", \"motorPHStartTime\":" + motorStates.motorPHStartTime
  + ", \"motorBrownAdjustSleepTime\":" + motorStates.motorBrownAdjustSleepTime
  + ", \"motorBrownAutoDosingStatus\":" + motorStates.motorBrownAutoDosingStatus
  + ", \"motorBrownRunningTime\":" + motorStates.motorBrownRunningTime
  + ", \"motorBrownSpeed\":" + motorStates.motorBrownSpeed
  + ", \"motorBrownStartTime\":" + motorStates.motorBrownStartTime
  + ", \"motorGreenAdjustSleepTime\":" + motorStates.motorGreenAdjustSleepTime
  + ", \"motorGreenAutoDosingStatus\":" + motorStates.motorGreenAutoDosingStatus
  + ", \"motorGreenRunningTime\":" + motorStates.motorGreenRunningTime
  + ", \"motorGreenSpeed\":" + motorStates.motorGreenSpeed
  + ", \"motorGreenStartTime\":" + motorStates.motorGreenStartTime
  + ", \"motorPinkAdjustSleepTime\":" + motorStates.motorPinkAdjustSleepTime
  + ", \"motorPinkAutoDosingStatus\":" + motorStates.motorPinkAutoDosingStatus
  + ", \"motorPinkRunningTime\":" + motorStates.motorPinkRunningTime
  + ", \"motorPinkSpeed\":" + motorStates.motorPinkSpeed
  + ", \"motorPinkStartTime\":" + motorStates.motorPinkStartTime      
  + "}}";


  Serial.println("[MQTT send JSON]=:>" + telemetryJSON);
  
  return "[MQTT send JSON]=:>" + telemetryJSON;

} // printSensorMetrics_Terminal()

// Configute the OLED SSD1306 Display
void configureSSD1306Display() {
  Serial.println(F("------------------- configureSSD1306Display ---------------------"));
  Serial.println(F("Adafruit SSD1306 display configure"));  

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    Serial.println(F("SSD1306 allocation success!"));
  }
  
  // display.cp437(true);         // Use full 256 char 'Code Page 437' font
  // Show initial display buffer contents on the screen --
  display.display();
  // Clear the display buffer
  display.clearDisplay();

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
// Initialise the I2C ports for the sensorts and motor controlled
void i2c_setup() {

  // Dosing Motor controller is conneted to I2C 0
  Wire.setClock(100000);
  Wire.begin();

  // All other sensors is conneted to I2C 1
  Wire1.begin();
} // end i2c_setup()


/* returns false if not supported */
// Print the Dallas DS18S20 OneWire tempreture sensor ID
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

// Print the Dallas DS18S20 OneWire tempreture sensor metrics scratchpad
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

// Configure the Dallas DS18S20 OneWire tempreture sensors
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

// Read the Dallas DS18S20 OneWire tempreture sensors
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
  
  delay(10000);
} // end readSensorsDS18S20()

// Configure the Raspberry Pi Pico ADC to read the Analogue PH sensor
void configureDFRobotPHv2(){
  Serial.println(F("------------------- configureDFRobotPHv2 ---------------------"));
  Serial.println(F("DFRobot PH V2 sensor configure")); 

  pinMode(PH_PIN,INPUT);
  //adc_gpio_init(PH_PIN);
  //adc_init();
  //adc_select_input(0); // 0 is GPIO26 A0
  analogReadResolution(12); // set analouge read resolution to 12 bits, default is 10
  //analogReadResolution(10); // set analouge read resolution to 12 bits, default is 10

  ph.begin();
  // values determined for used copy of the pH probe, need to recalibrate once per month to be safe
  //ph.initVoltages(1508.96, 2033.13);
  ph.initVoltages(1583.61, 2090.5);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
} // 

float readReservoirWaterTemperature()
{
  return sensorMetrics.ds18b20TempOutlet; // make sure this is set before calling readDFRobotPHv2()
}

// Read the Raspberry Pi Pico ADC PH Pin Value to read the Analogue PH sensor
void readDFRobotPHv2()
{
  Serial.println(F("------------------- readDFRobotPHv2 ---------------------"));
  Serial.println(F("DFRobot PH V2 sensor read"));    

  sensorMetrics.valuePHVoltageUINT16 = analogRead(PH_PIN); // raw ADC value
  sensorMetrics.valuePHVoltage = sensorMetrics.valuePHVoltageUINT16/4096.0*3253;  // read the voltage /1024.0 for 10 bit ADC or /4096 for 12 bit ADC and also  *5000 for 5V system and *3300 3.3 V system ours measure 3.253V
  sensorMetrics.valuePH = ph.readPH(sensorMetrics.valuePHVoltage, readReservoirWaterTemperature());  // convert voltage to pH with temperature compensation
  Serial.print("temperature: ");
  Serial.print(readReservoirWaterTemperature(),1);
  Serial.print(" ^C  pH: ");
  Serial.println(sensorMetrics.valuePH, 2);
  Serial.print("valuePHVoltageUINT16: ");
  Serial.println(sensorMetrics.valuePHVoltageUINT16);  
  Serial.print("phVoltage: ");
  Serial.println(sensorMetrics.valuePHVoltage, 2);
//  ph.calibration(sensorMetrics.valuePHVoltage, readReservoirWaterTemperature());           // calibration process by Serail CMD

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
}

// Configure the Raspberry Pi Pico ADC to read the Analogue EC k1 sensor
void configureDFRobotECk1(){
  Serial.println(F("------------------- configureDFRobotECk1 ---------------------"));
  Serial.println(F("DFRobot EC K1 sensor configure")); 

  pinMode(EC_PIN,INPUT);
  analogReadResolution(12); // set analouge read resolution to 12 bits, default is 10

  ec.begin();

  // values determined for used copy of the EC probe, need to recalibrate once per month to be safe
  ec.initKValues(1.07, 12.88);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
} // 

// Read the Raspberry Pi Pico ADC EC pin to read the Analogue EC k1 sensor
void readDFRobotECk1()
{
  Serial.println(F("------------------- readDFRobotECk1 ---------------------"));
  Serial.println(F("DFRobot EC K1 sensor read"));    

  sensorMetrics.valueECVoltageUINT16 = analogRead(EC_PIN); // raw ADC value
  sensorMetrics.valueECVoltage = sensorMetrics.valueECVoltageUINT16/4096.0*3253;  // read the voltage /1024.0 for 10 bit ADC or /4096 for 12 bit ADC and also  *5000 for 5V system and *3300 3.3 V system ours measure 3.253V
  sensorMetrics.valueEC = ec.readEC(sensorMetrics.valueECVoltage, readReservoirWaterTemperature());  // convert voltage to EC with temperature compensation
  Serial.print("temperature: ");
  Serial.print(readReservoirWaterTemperature(),1);
  Serial.print(" ^C  EC: ");
  Serial.println(sensorMetrics.valueEC, 2);
  Serial.print("ms/cm valueECVoltageUINT16: ");
  Serial.println(sensorMetrics.valueECVoltageUINT16);  
  Serial.print("ecVoltage: ");
  Serial.println(sensorMetrics.valueECVoltage, 2);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
}


void sync_time_sntp() {
//  setTime(WiFi.getTime() + (SECS_PER_HOUR * TIME_ZONE));
}

void refreshTime() {
  Serial.println();
  Serial.println(F("------------------------- refreshTime  -----------------------"));

  Serial.println("refreshTime: timeEpochms : " + timeEpochms);
  timeEpoch = String(timeInfo.tm_year + 1900) + "-" + String(timeInfo.tm_mon + 1) + "-" + String(timeInfo.tm_mday) + "-" +  String(timeInfo.tm_hour) + "-" + String(timeInfo.tm_min) + "-" +  String(timeInfo.tm_sec);
  Serial.println(F("--------------------------------------------------------------"));
  Serial.println();
}

void printTime() {
  Serial.println();
  Serial.println(F("------------------------- printTime ---------------------------"));
  //refreshTime(); // refresh in main loop to keep time the same for all sensor and photo readings
  Serial.print(F("year:"));
  Serial.print(timeInfo.tm_year + 1900); // years since 1900
  Serial.print(F("\tmonth:"));
  Serial.print(timeInfo.tm_mon + 1); // January = 0 (!)
  Serial.print(F("\tday:"));
  Serial.print(timeInfo.tm_mday); // day of month
  Serial.print(F("\thour:"));
  Serial.print(timeInfo.tm_hour); // hours since midnight 0-23
  Serial.print(F("\tmin:"));
  Serial.print(timeInfo.tm_min); // minutes after the hour 0-59
  Serial.print(F("\tsec:"));
  Serial.print(timeInfo.tm_sec); // seconds after the minute 0-61*
  Serial.print(F("\twday"));
  Serial.print(timeInfo.tm_wday); // days since Sunday 0-6
  if (timeInfo.tm_isdst == 1) // Daylight Saving Time flag
    Serial.print(F("\tDST"));
  else
    Serial.print(F("\tstandard"));
  Serial.println();
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();
} // end printTime() 

void printTime2() {
  Serial.println();
  Serial.println(F("------------------------- printTime2 ---------------------------"));  
  char buff[20];
  sprintf(buff, "%02d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
  Serial.println(buff);
  Serial.println();
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();  
}


void configureRelayBoard(){
    pinMode(PIN_RELAY_1, OUTPUT); // set onboard led pin to output
    pinMode(PIN_RELAY_2, OUTPUT); // set onboard led pin to output
    pinMode(PIN_RELAY_3, OUTPUT); // set onboard led pin to output
    pinMode(PIN_RELAY_4, OUTPUT); // set onboard led pin to output
}

void relay1ON(){
  digitalWrite(PIN_RELAY_1, HIGH);
}
void relay1OFF(){
  digitalWrite(PIN_RELAY_1, LOW);
}

void relay2ON(){
  digitalWrite(PIN_RELAY_2, HIGH);
}
void relay2OFF(){
  digitalWrite(PIN_RELAY_2, LOW);
}

void relay3ON(){
  digitalWrite(PIN_RELAY_3, HIGH);
}
void relay3OFF(){
  digitalWrite(PIN_RELAY_3, LOW);
}

void relay4ON(){
  digitalWrite(PIN_RELAY_4, HIGH);
}
void relay4OFF(){
  digitalWrite(PIN_RELAY_4, LOW);
}


void relayControllerTestLoop() {

  Serial.println(F("------------------- relayTestLoop ---------------------"));
  Serial.println(F("Wave RelayController Test")); 

  int relayCMD = 0; 

  Serial.println("Select Option:"); 
  Serial.println("1: Relay 1 ON"); 
  Serial.println("2: Relay 1 OFF"); 
  Serial.println("3: Relay 2 ON"); 
  Serial.println("4: Relay 2 OFF"); 
  Serial.println("5: Relay 3 ON"); 
  Serial.println("6: Relay 3 OFF"); 
  Serial.println("7: Relay 4 ON"); 
  Serial.println("8: Relay 4 OFF");   
  Serial.println("9: Exit"); 

  //Serial.setTimeout(30000); // serail read timeout is 30s


  while (relayCMD >= 0 and relayCMD < 9)
  {
    if(Serial.available() > 0) {
      relayCMD = Serial.parseInt(); // returns 0 after serial timeout
      Serial.println(relayCMD);
    } else {
      relayCMD = 0;
    }

    // speed 68 = 0.2 ml/s
    // speed 108 = 0.5 ml/s
    // speed 182 = 1 ml/s  
    switch(relayCMD){
      case 1:
        relay1ON();
        break;
      case 2:
        relay2ON();
        break;
      case 3:
        relay3ON();
        break;
      case 4:
        relay4ON();
        break;
      case 5:
        relay1OFF();
        break;
      case 6:
        relay2OFF();
        break;
      case 7:
        relay3OFF();
        break;
      case 8:
        relay4OFF();
        break;
    }

    yield();
  } // while icdm 0..5        
  
  //Serial.setTimeout(1000); // set serail read timeout back to default

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();    
} // relayControllerTestLoop



void configureMotorShield() {
  Serial.println(F("------------------- configureMotorShield ---------------------"));
  Serial.println(F("Wave MotorShield configure")); 

  if (!AFMS.begin(1600, &Wire)) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
} // 


void motorPHDownStart(uint8_t mSpeed, uint8_t mDirection, uint16_t runningTime) {
  motorStates.motorPHSpeed = mSpeed;
  motorStates.motorPHDirection = mDirection;
  motorStates.motorPHRunningTime = runningTime;
  motorStates.motorPHStartTime =  millis();
  //motorPH->fullOn();
  motorPH->run(mDirection);
  motorPH->setSpeed(mSpeed);
  Serial.println("Priming motorPH Started");
} // motorPHDownStart

void motorPHDownStop() {
  if(motorStates.motorPHSpeed > 0){
    motorPH->run(RELEASE);
    motorPH->setSpeed(0);
    motorStates.motorPHSpeed = 0;
    Serial.println("Priming motorPH Stopped");
  }
  motorStates.motorPHDirection = 0;
  motorStates.motorPHRunningTime = 0;
  motorStates.motorPHStartTime =  0;
} // motorPHDownStop

void motorPHDownRun() {
  // stop the motor after timeout
  if((millis() - motorStates.motorPHStartTime) > motorStates.motorPHRunningTime*1000) { 
    motorPHDownStop();
  }
} // motorPHDownRun


void motorBrownDoseStart(uint8_t mSpeed, uint8_t mDirection, uint16_t runningTime) {
  motorStates.motorBrownSpeed = mSpeed;
  motorStates.motorBrownDirection = mDirection;
  motorStates.motorBrownRunningTime = runningTime;
  motorStates.motorBrownStartTime =  millis();
  //motorBrown->fullOn();
  motorBrown->run(mDirection);
  motorBrown->setSpeed(mSpeed);
  Serial.println("Priming motorBrown Started");
} // motorBrownDownStart

void motorBrownDoseStop() {
  if(motorStates.motorBrownSpeed>0){
    motorBrown->run(RELEASE);
    motorBrown->setSpeed(0);
    motorStates.motorBrownSpeed = 0;
    Serial.println("Priming motorBrown Stopped");
  }
  motorStates.motorBrownDirection = 0;
  motorStates.motorBrownRunningTime = 0;
  motorStates.motorBrownStartTime =  0;
} // motorPHDownStop

void motorBrownDoseRun() {
  // stop the motor after timeout
  if((millis() - motorStates.motorBrownStartTime) > motorStates.motorBrownRunningTime*1000) { 
    motorBrownDoseStop();
  }
} // motorBrownDownRun


void motorGreenDoseStart(uint8_t mSpeed, uint8_t mDirection, uint16_t runningTime) {
  motorStates.motorGreenSpeed = mSpeed;
  motorStates.motorGreenDirection = mDirection;
  motorStates.motorGreenRunningTime = runningTime;
  motorStates.motorGreenStartTime =  millis();
  //motorGreen->fullOn();
  motorGreen->run(mDirection);
  motorGreen->setSpeed(mSpeed);
  Serial.println("Priming motorGreen Started");
} // motorGreenDownStart

void motorGreenDoseStop() {
  if(motorStates.motorGreenSpeed>0){
    motorGreen->run(RELEASE);
    motorGreen->setSpeed(0);
    motorStates.motorGreenSpeed = 0;
    Serial.println("Priming motorGreen Stopped");
  }
  motorStates.motorGreenDirection = 0;
  motorStates.motorGreenRunningTime = 0;
  motorStates.motorGreenStartTime =  0;
} // motorGreenDownStop

void motorGreenDoseRun() {
  // stop the motor after timeout
  if((millis() - motorStates.motorGreenStartTime) > motorStates.motorGreenRunningTime*1000) { 
    motorGreenDoseStop();
  }
} // motorGreenDownRun


void motorPinkDoseStart(uint8_t mSpeed, uint8_t mDirection, uint16_t runningTime) {
  motorStates.motorPinkSpeed = mSpeed;
  motorStates.motorPinkDirection = mDirection;
  motorStates.motorPinkRunningTime = runningTime;
  motorStates.motorPinkStartTime =  millis();
  //motorPink->fullOn();
  motorPink->run(mDirection);
  motorPink->setSpeed(mSpeed);
  Serial.println("Priming motorPink Started");
} // motorPinkDownStart

void motorPinkDoseStop() {
  if(motorStates.motorPinkSpeed>0){
    motorPink->run(RELEASE);
    motorPink->setSpeed(0);
    motorStates.motorPinkSpeed = 0;    
    Serial.println("Priming motorPink Stopped");
  }
  motorStates.motorPinkDirection = 0;
  motorStates.motorPinkRunningTime = 0;
  motorStates.motorPinkStartTime =  0;
} // motorPinkDownStop

void motorPinkDoseRun() {
  // stop the motor after timeout
  if((millis() - motorStates.motorPinkStartTime) > motorStates.motorPinkRunningTime*1000) { 
    motorPinkDoseStop();
  }
} // motorPinkDownRun

// Dosing Motor run timing loop, stop started motors when timeout reached
void motorsDosingRun() {
  motorPHDownRun();
  motorBrownDoseRun();
  motorGreenDoseRun();
  motorPinkDoseRun();
  yield();
} // motorsDosingRun


void motorTestLoop() {

  Serial.println(F("------------------- motorTestLoop ---------------------"));
  Serial.println(F("Wave MotorShield Motor Test")); 

  int motorCMD = 0; 

  Serial.println("Select Motor:"); 
  Serial.println("1: PH motor Priming"); 
  Serial.println("2: Brown Flued Priming"); 
  Serial.println("3: Green Flued Priming"); 
  Serial.println("4: Pink Flued Priming"); 
  Serial.println("9: Exit"); 

  //Serial.setTimeout(30000); // serail read timeout is 30s

  while (motorCMD >= 0 and motorCMD < 9)
  {
    if(Serial.available() > 0) {
      motorCMD = Serial.parseInt(); // returns 0 after serial timeout
      Serial.println(motorCMD);
    } else {
      motorCMD = 0;
    }

    // speed 68 = 0.2 ml/s
    // speed 108 = 0.5 ml/s
    // speed 182 = 1 ml/s  
    switch(motorCMD){
      case 1:
        motorPHDownStart(68, FORWARD, 500);
        Serial.println("Priming motorPH");
        break;
      case 2:
        motorBrownDoseStart(100, FORWARD, 80);
        Serial.println("Priming motorBrown");
        break;
      case 3:
        motorGreenDoseStart(108, FORWARD, 200); 
        Serial.println("Priming motorGreen");
        break;
      case 4:
        motorPinkDoseStart(182, FORWARD, 80);
        Serial.println("Priming motorPink");
        break;                                
    }

    yield();
  } // while icdm 0..5        
  
  //Serial.setTimeout(1000); // set serail read timeout back to default

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();    
} // motorTestLoop

// Switch specific Control Relay On or Off based on recieved MQTT or Serial command
void triggerRelay(uint8_t relayNO, String relayStatus){
    Serial.println("triggerRelay ==> ");
    Serial.println("relayNO ==> " + relayNO);
    Serial.println("relayStatus ==> " + relayStatus);

    switch(relayNO){
      case 1:
        if(relayStatus.equals("ON")){
         relay1ON(); 
        }else{
         relay1OFF();
        }        
        break;
      case 2:
        if(relayStatus.equals("ON")){
         relay2ON(); 
        }else{
         relay2OFF();
        }        
       break;        
      case 3:
        if(relayStatus.equals("ON")){
         relay3ON(); 
        }else{
         relay3OFF();
        }  
        break;
      case 4:
        if(relayStatus.equals("ON")){
         relay4ON(); 
        }else{
         relay4OFF();
        }        
        break;
    } // relayNO
} // triggerRelay

// Start Dosing Motor based on recieved MQTT command
void triggerDosingMotor(String motorNO, String motorStatus, uint8_t speed, uint16_t runTime){
    Serial.println("triggerDosingMotor ==> ");
    Serial.println("motorNO ==> " + motorNO);
    Serial.println("motorStatus ==> " + motorStatus);  

  if(motorNO.equals("motorPH")){
      if(motorStatus.equals("ON")){ 
        motorPHDownStart(speed, FORWARD, runTime);
      } else {
        motorPHDownStop();
      }
  } else if(motorNO.equals("motorBrown")){
      if(motorStatus.equals("ON")){ 
        motorBrownDoseStart(speed, FORWARD, runTime);
      } else {
        motorBrownDoseStop();
      }
  } else if(motorNO.equals("motorGreen")){ 
      if(motorStatus.equals("ON")){ 
        motorGreenDoseStart(speed, FORWARD, runTime);
      } else {
        motorGreenDoseStop();
      }
  } else if(motorNO.equals("motorPink")){ 
      if(motorStatus.equals("ON")){ 
        motorPinkDoseStart(speed, FORWARD, runTime);
      } else {
        motorPinkDoseStop();
      }
  }
} // triggerDosingMotor

void setAutoDosingStatus(String motorNO, String autoDosingStatus){
    Serial.println("setAutoDosingStatus ==> ");
    Serial.println("motorNO ==> " + motorNO);
    Serial.println("autoDosingStatus ==> " + autoDosingStatus);  

  if(motorNO.equals("motorPH")){
      if(autoDosingStatus.equals("ON")){ 
        motorStates.motorPHAutoDosingStatus = 1;
      } else {
        motorStates.motorPHAutoDosingStatus = 0;
      }
  } else if(motorNO.equals("motorBrown")){
      if(autoDosingStatus.equals("ON")){ 
        motorStates.motorBrownAutoDosingStatus = 1;
      } else {
        motorStates.motorBrownAutoDosingStatus = 0;
      }
  } else if(motorNO.equals("motorGreen")){ 
      if(autoDosingStatus.equals("ON")){ 
        motorStates.motorGreenAutoDosingStatus = 1;
      } else {
        motorStates.motorGreenAutoDosingStatus = 0;
      }
  } else if(motorNO.equals("motorPink")){ 
      if(autoDosingStatus.equals("ON")){ 
        motorStates.motorPinkAutoDosingStatus = 1;
      } else {
        motorStates.motorPinkAutoDosingStatus = 0;
      }
  } else if(motorNO.equals("motorALL")){ 
      if(autoDosingStatus.equals("ON")){ 
        motorStates.motorPHAutoDosingStatus = 1;
        motorStates.motorBrownAutoDosingStatus = 1;
        motorStates.motorGreenAutoDosingStatus = 1;
        motorStates.motorPinkAutoDosingStatus = 1;
      } else {
        motorStates.motorPHAutoDosingStatus = 0;
        motorStates.motorBrownAutoDosingStatus = 0;
        motorStates.motorGreenAutoDosingStatus = 0;
        motorStates.motorPinkAutoDosingStatus = 0;
      }
  }

} // setAutoDosingStatus


void mqttJSONMessageProcess(String pJSONStr){
  Serial.println("mqttMessageProcess ==> " + pJSONStr);
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, pJSONStr);

  //const char* sensor = doc["action"];
  String actionStr = doc["action"];

  if(actionStr.startsWith("relayControl")){
    uint8_t relayNO = doc["relayNo"];
    String relayStatus = doc["relayStatus"];
    triggerRelay(relayNO, relayStatus);
  } else if(actionStr.startsWith("motorControl")){
    String motorNO = doc["motorNO"];
    String motorStatus = doc["motorStatus"];
    uint8_t speed = doc["speed"];
    uint16_t runTime = doc["runTime"];
    triggerDosingMotor(motorNO, motorStatus, speed, runTime);
  } else if(actionStr.startsWith("autoDosing")){
    String motorNO = doc["motorNO"];
    String autoDosingStatus = doc["autoDosingStatus"];
    setAutoDosingStatus(motorNO, autoDosingStatus);
  }

} // mqttJSONMessageProcess


String serialRxStr = "";  //read until timeout
byte serialCMDmodeIndex = 0;

byte cmdParse(String *pStr)
{
    byte serialCMDmodeIndex = 0;

    if(pStr->startsWith("[MQTT Message arrived]:=>")){
        serialCMDmodeIndex = 1;
        Serial.println("RECEIVED MQTT SERIAL CMD :==>" + *pStr);
        String jsonStr = pStr->substring(59);
        mqttJSONMessageProcess(jsonStr);
    }else if(pStr->startsWith("[RECIEVE TIME EPOCH MS]=:>")){
        serialCMDmodeIndex = 2;
        Serial.println("[RECIEVE TIME EPOCH MS]=:>" + timeEpochms);  

    }else if(pStr->startsWith("ENTERPH") || pStr->startsWith("CALPH") || pStr->startsWith("EXITPH")){
        serialCMDmodeIndex = 3;
        char cmd_array[pStr->length()];
        pStr->toCharArray(cmd_array, pStr->length());
        ph.calibration(sensorMetrics.valuePHVoltage, readReservoirWaterTemperature(), cmd_array); // calibration process by Serail CMD
    }else if(pStr->startsWith("ENTEREC") || pStr->startsWith("CALEC") || pStr->startsWith("EXITEC")){
        serialCMDmodeIndex = 4;
        char cmd_array[pStr->length()];
        pStr->toCharArray(cmd_array, pStr->length());        
        ec.calibration(sensorMetrics.valueECVoltage, readReservoirWaterTemperature(), cmd_array); // calibration process by Serail CMD      
    }else if(pStr->startsWith("MOTORTEST")){
        serialCMDmodeIndex = 5;
        motorTestLoop();
    }else if(pStr->startsWith("RELAYTEST")){
        serialCMDmodeIndex = 6;
        relayControllerTestLoop();
    }

    return serialCMDmodeIndex;
} // cmdParse


void serialEventProcess() {  
  if(Serial.available() > 0){
    Serial.println();
    Serial.println(F("---------------------- serialEvent --------------------------"));

    serialRxStr = Serial.readString();  //read until timeout
    serialRxStr.trim();  // remove any \r \n whitespace at the end of the String
    Serial.println("serialRxStr :> " + serialRxStr);  
    cmdParse(&serialRxStr);
    Serial.println(F("---------------------------------------------------------------"));
    Serial.println();  

  }
  //yield();
} // serialEvent()

String serialMQTTRxStr = "";  //read until timeout
void serial1MQTTRXEventProcess() {  
  if(Serial1.available() > 0){
    Serial.println();
    Serial.println(F("----------------- serial1MQTTRXEventProcess --------------------"));

    serialMQTTRxStr = Serial1.readString();  //read until timeout
    serialMQTTRxStr.trim();  // remove any \r \n whitespace at the end of the String
    Serial.println("serialMQTTRxStr :> " + serialMQTTRxStr);  
    cmdParse(&serialMQTTRxStr);
    Serial.println(F("---------------------------------------------------------------"));
    Serial.println();  

  }
  //yield();
} // serialEvent()


unsigned long previousMQTTSendMillis = 0;

void sendSensorMetrics() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMQTTSendMillis > MQTT_DATA_SEND_INTERVAL) { // Prevent main loop blocking but run this at interval
    previousMQTTSendMillis = currentMillis;
    Serial1.println(getSensorMetrics_JSON());
  }
  serial1MQTTRXEventProcess();
  yield(); // give other processes a chance
} // sendSensorMetrics()


/*
 Auto adjust the PH down when it gets too high i.e. above 6.5
 It must be between 5.5 and 6.5 but prefarably around 6.0 for the plants we plant 
 The water tank automaticallay fills with tap water float valve when it
 gets low which increases the PH due to tapwater PH of around 8 thus 
 downwards adjustment is required.
*/
void phDownMilliLiter(uint8_t pMilliliters) {
  uint16_t runSeconds = pMilliliters * (1/0.2);  // required millilitres * 0.2 millilitres per second pumping rate
  motorPHDownStart(68, FORWARD, runSeconds); // motor speed 68 delivers 0.2 milliliters per second
}

void phAdjustDown(uint8_t tankVolume, float currentPH, float targetPH){
 if(currentPH > 0 && currentPH > targetPH && motorStates.motorPHAutoDosingStatus == 1 ) {
    phDownMilliLiter(5); // add 5 milliliters of ACID PH Down liquid on each adjustment run, need to do better algorithm
 }
} // phAdjustDown


/*
 Auto adjust the EC up when it gets too low, i.e. below 1450
 It must be between 1450 and 1600 but prefarably around 1500 for the plants we plant 
 The water tank automaticallay fills with tap water float valve when it gets low.
 THis decreases plant nutrient level concentration and the plants also use the nutrients and thus EC decreases.
*/
void brownDownMilliLiter(uint8_t pMilliliters) {
  uint16_t runSeconds = pMilliliters;  // required millilitres * 1 millilitres per second pumping rate
  motorBrownDoseStart(182, FORWARD, runSeconds); // motor speed 182 delivers 1 milliliters per second
}

void brownAdjustUp(uint8_t tankVolume, float currentEC, float targetEC){
 if(currentEC > 0 && currentEC < targetEC && motorStates.motorBrownAutoDosingStatus == 1 ) {
    brownDownMilliLiter(25); // add 25 milliliters of Brown food liquid on each adjustment run, need to do better algorithm
 }
} // brownAdjustUp


/*
 Auto adjust the EC up when it gets too low, i.e. below 1450
 It must be between 1450 and 1600 but prefarably around 1500 for the plants we plant 
 The water tank automaticallay fills with tap water float valve when it gets low.
 THis decreases plant nutrient level concentration and the plants also use the nutrients and thus EC decreases.
*/
void greenDownMilliLiter(uint8_t pMilliliters) {
  uint16_t runSeconds = pMilliliters;  // required millilitres * 1 millilitres per second pumping rate
  motorGreenDoseStart(182, FORWARD, runSeconds); // motor speed 182 delivers 1 milliliters per second
}

void greenAdjustUp(uint8_t tankVolume, float currentEC, float targetEC){
 if(currentEC > 0 && currentEC < targetEC && motorStates.motorGreenAutoDosingStatus == 1 ) {
    greenDownMilliLiter(25); // add 25 milliliters of Green food liquid on each adjustment run, need to do better algorithm
 }
} // greenAdjustUp


/*
 Auto adjust the EC up when it gets too low, i.e. below 1450
 It must be between 1450us/cm and 1600us/cm but prefarably around 1550us/cm for the plants we plant 
 The water tank automaticallay fills with tap water float valve when it gets low.
 THis decreases plant nutrient level concentration and the plants also use the nutrients and thus EC decreases.
*/
void pinkDownMilliLiter(uint8_t pMilliliters) {
  uint16_t runSeconds = pMilliliters;  // required millilitres * 1 millilitres per second pumping rate
  motorPinkDoseStart(182, FORWARD, runSeconds); // motor speed 182 delivers 1 milliliters per second
}

void pinkAdjustUp(uint8_t tankVolume, float currentEC, float targetEC){
 if(currentEC > 0 && currentEC < targetEC && motorStates.motorPinkAutoDosingStatus == 1 ) {
    pinkDownMilliLiter(25); // add 25 milliliters of Pink food liquid on each adjustment run, need to do better algorithm
 }
} // pinkAdjustUp


// Auto Adjust the PH and EC every 10 minutes if need to
void autoDosingMotorLoop() {
  unsigned long currentMillis = millis();

  if(sensorMetrics.valueEC > 0 && sensorMetrics.valuePH > 0) { // make sure that we have readings before we adjust

    if (currentMillis - motorStates.motorPHAdjustSleepTime > AUTO_ADJUST_PH_INTERVAL) { // Prevent main loop blocking but run this at interval
      motorStates.motorPHAdjustSleepTime = currentMillis;
      phAdjustDown(200, sensorMetrics.valuePH , AUTO_ADJUST_PH_LEVEL);
    }

    if (currentMillis - motorStates.motorBrownAdjustSleepTime > AUTO_ADJUST_BROWN_INTERVAL) { // Prevent main loop blocking but run this at interval
      motorStates.motorBrownAdjustSleepTime = currentMillis;
      brownAdjustUp(200, sensorMetrics.valueEC , AUTO_ADJUST_EC_LEVEL);
    }

    if (currentMillis - motorStates.motorGreenAdjustSleepTime > AUTO_ADJUST_GREEN_INTERVAL) { // Prevent main loop blocking but run this at interval
      motorStates.motorGreenAdjustSleepTime = currentMillis;
      greenAdjustUp(200, sensorMetrics.valueEC , AUTO_ADJUST_EC_LEVEL);
    }

    if (currentMillis - motorStates.motorPinkAdjustSleepTime > AUTO_ADJUST_PINK_INTERVAL) { // Prevent main loop blocking but run this at interval
      motorStates.motorPinkAdjustSleepTime = currentMillis;
      pinkAdjustUp(200, sensorMetrics.valueEC , AUTO_ADJUST_EC_LEVEL);
    }

  } // end if
  
  yield(); // give other processes a chance
}



void blinkPicoLED(){
  // Serial.println("RapBerry Pi Pico saying: LED On");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

void logPrintln(String &pStr){
  Serial.println(pStr);
}

void logPrint(String &pStr){
  Serial.print(pStr);
}

void scheduledRestart(){
  delay(86400000U); // pause for 24hours and then restart the system
  Serial.println("[RESTARTING SYSTEM]");
  //restart_system();
}



void setup() {

  delay(2000); 
  // put your setup code here, to run once:
  Serial.begin(115200); //  Start the serial port for terminal communication
  Serial1.begin(115200);

  //while (!Serial);
  while (!Serial1);
  
  delay(2000); 

  Serial.println("---------------------------- SETUP -------------------------------");

  //sync_time_sntp(); // Sync time with time server

  pinMode(LED_BUILTIN, OUTPUT); // set onboard led pin to output
  configureRelayBoard(); 

  i2c_setup(); // setup i2c bus
  delay(2000); // give I2C time to initialise otherwise PICO wont boot

  //pinMode(DHTPIN, INPUT_PULLUP); // set in driver lib now, no deed anymore
  dht_sensor_init(); // Initilaise the DHT11 Temperature and Humitiy sensor
  dht_sensor_info_print();
  configurSensorsDS18S20(); // Configure the water reservior Inlet and Outlet temperature sensors

  // configure I2C sensors
  configureSSD1306Display();
  configureCSS811Sensor(); // set up CSS811 Air Quality sensor
  setCSS811BaseLine(); // needs to bo done once every 24h for new sensor and once a mont for burned in sensor
  configureTSL2591Sensor(); // Initilaise TSL2591 Light Sensor
  #ifdef DEBUG_ENABLED
   displayTSL2591SensorDetails();
  #endif
  configureLTR390sensor(); // set up TR390 UV sensor
  configureAS7341sensor(); // set up colour spectrum intensity sensor
  configureBME680(); // set up the DFRobot BME680 environment sensor
  configureXENSIVPAS2(); // set up Infineon XENSIV PAS2 CO2 sensor
  configureDFRobotOxygenSensor();

  configureDFRobotPHv2();
  configureDFRobotECk1();

  configureMotorShield();

  Scheduler.startLoop(blinkPicoLED);
  Scheduler.startLoop(motorsDosingRun);
  Scheduler.startLoop(autoDosingMotorLoop);
  Scheduler.startLoop(sendSensorMetrics); //  send metrics to AWS IoT and ThingsBoard
  Scheduler.startLoop(scheduledRestart); // reboot the system once every 24 hours
  //Scheduler.startLoop(serialEventProcess);

  Serial.println("---------------------------------------------------------------");
  Serial.println();   
} // end setup()



unsigned long previousMillis = 0;

void loop() {
  // put your main code here, to run repeatedly:
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > SENSORS_READ_INTERVAL) { // Prevent main loop blocking but run this at interval
    previousMillis = currentMillis;

      // printTime2();
      // Read Air Metrics sensors
      dht_sensor_read_print();
      readCSS811Sensor();
      readBME680();
      readXENSIVPAS2();
      readDFRobotOxygenSensor();

      // Read Light etrics sensors
      advancedReadTSL2591Sensor();
      readLTR390Sensor();
      readAS7341sensor();

      // Read Nurtrient flued Metrics sensors
      readSensorsDS18S20();    
      if(serialCMDmodeIndex != 3 && serialCMDmodeIndex !=4) { // do not read when in calibration mode
        readDFRobotPHv2();
        readDFRobotECk1();
      }
      
      //Present measure Metrics sensor data
      printSensorMetrics_Terminal();
      printSensorMetrics_SSD1306();
      //sendSensorMetrics(); // added as scheduler send metrics to AWS IoT and ThingsBoard     
  }

  serialEventProcess();

  yield();
  //delay(100);

} // end loop()

