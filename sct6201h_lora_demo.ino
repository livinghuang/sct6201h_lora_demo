/*
This example code is modified from  Heltec Lora Basic sample code
It could run device as Rx or Tx by define DEVICE_AS_RX or not define
*/

#define DEVICE_AS_RX // set device as RX device , or TX device

#include "sct62_bsp.h"
#include "Wire.h"
#include "BMP280.h"
#include "HDC1080.h"
#include "LoRaWan_APP.h"
#include "Arduino.h"

#define RF_FREQUENCY 915000000 // Hz

#define TX_OUTPUT_POWER 22 // dBm

#define LORA_BANDWIDTH 0        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 8 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 256 // Define the payload size here

uint64_t Sleep_uSec = 60 * 1000000; // unit : uSec

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle = true;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);

RTC_DATA_ATTR int bootCount = 0;
BMP280 bmp;
HDC1080 hdc1080;
float getBatVolt();
uint8_t GetBatteryLevel(void);
struct hdc1080_data
{
  float temperature;
  float humidity;
} hdc1080_result;

struct bmp280_data
{
  float bmp280_internal_temperature;
  float pressure;
} bmp280_result;

bool hdc1080_fetch(void)
{
  if (!hdc1080.begin())
  {
    return 0;
  }
  float temp = hdc1080.readTemperature();
  float Humidity = hdc1080.readHumidity();
  temp = hdc1080.readTemperature();
  Humidity = hdc1080.readHumidity();
  hdc1080.end();
  // Serial.printf("T=%.2f degC, Humidity=%.2f %\n", temp, Humidity);
  hdc1080_result.temperature = temp;
  hdc1080_result.humidity = Humidity;
  return 1;
}

bool bmp280_fetch(void)
{
  if (!bmp.begin())
  {
    return 0;
  }
  delay(50);
  bmp.setSampling(BMP280::MODE_NORMAL,     /* Operating Mode. */
                  BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  BMP280::FILTER_X16,      /* Filtering. */
                  BMP280::STANDBY_MS_500); /* Standby time. */
  delay(50);
  float temp = bmp.readTemperature();
  float Pressure = (float)bmp.readPressure() / 100.0;
  delay(100);
  int c = 0;
  while ((temp < -50) || (Pressure > 1100) || (Pressure < 500))
  {
    bmp.putBMP280ToSleep();
    delay(10);
    bmp.end();
    Serial.println("BMP ERROR");
    Serial.flush();
    delay(100);
    bmp.begin();
    delay(100);
    bmp.setSampling(BMP280::MODE_NORMAL,     /* Operating Mode. */
                    BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    BMP280::FILTER_X16,      /* Filtering. */
                    BMP280::STANDBY_MS_500); /* Standby time. */
    temp = bmp.readTemperature();
    Pressure = (float)bmp.readPressure() / 100.0;
    c++;
    if (c > 3)
    {
      return false;
    }
  }
  bmp.putBMP280ToSleep();
  delay(100);
  bmp.end();
  // Serial.printf("T=%.2f degC, Pressure=%.2f hPa\n", temp, Pressure);
  bmp280_result.bmp280_internal_temperature = temp;
  bmp280_result.pressure = Pressure;
  return true;
}

void power_On_Sensor_Bus()
{
  pinMode(pVext, OUTPUT);
  // according the I2C bus collision with the JTAG/USB pin , it is internal pull up, we need to set it high before turn on the Vext to avoid system restart
  pinMode(pSDA, OUTPUT);
  pinMode(pSCL, OUTPUT);
  digitalWrite(pSDA, HIGH);
  digitalWrite(pSCL, HIGH);
  delay(15);
  digitalWrite(pVext, LOW);
}

void power_Off_Sensor_Bus()
{
  pinMode(pVext, OUTPUT);
  // according the I2C bus collision with the JTAG/USB pin , it is internal pull down, we need to set it low before turn off the Vext to keep the Vext level in low.
  pinMode(pSDA, OUTPUT);
  pinMode(pSCL, OUTPUT);
  digitalWrite(pSDA, LOW);
  digitalWrite(pSCL, LOW);
  delay(15);
  digitalWrite(pVext, HIGH);
}

void fetchSensorData()
{
  int i;
  for (i = 0; i < 3; i++)
  {
    power_On_Sensor_Bus();
    delay(15);
    hdc1080_fetch();
    delay(10);
    power_Off_Sensor_Bus();
    delay(15);
  }

  for (i = 0; i < 3; i++)
  {
    power_On_Sensor_Bus();
    delay(10);
    bmp280_fetch();
    // delay(10);
    power_Off_Sensor_Bus();
    delay(5);
  }
}

float getBatVolt()
{
  uint32_t sum = 0;
  uint32_t test_min = 695;
  uint32_t test_max = 1030;
  for (size_t i = 0; i < 16; i++)
  {
    sum += analogRead(2);
    delay(10);
  }
  float avg = (float)(sum >> 4) / 4095 * 2400;
  Serial.print("avg");
  Serial.println(avg);
  return ((avg - test_min) * (4.2 - 3) / (test_max - test_min) + 3);
}

uint8_t GetBatteryLevel(void)
{
  const float maxBattery = 4.2;
  const float minBattery = 3.0;
  const float batVolt = getBatVolt();
  const float batVoltage = fmax(minBattery, fmin(maxBattery, batVolt));
  uint8_t batLevel = BAT_LEVEL_EMPTY + ((batVoltage - minBattery) / (maxBattery - minBattery)) * (BAT_LEVEL_FULL - BAT_LEVEL_EMPTY);
  if (batVolt > 4.2)
  {
    batLevel = 255;
  }
  if (batVolt < 3.01)
  {
    batLevel = 0;
  }
  Serial.print("{");
  Serial.println(batVoltage);
  Serial.print(batLevel);
  Serial.println("}");
  return batLevel;
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

static void prepareTxFrame(uint8_t port)
{
  bool rst = 0;
  appDataSize = sizeof(hdc1080_data) + sizeof(bmp280_data) + 1;

  fetchSensorData();
  Serial.println("Fetch data Done");
  Serial.printf("T=%.2f degC, Pressure=%.2f hPa\n", bmp280_result.bmp280_internal_temperature, bmp280_result.pressure);
  Serial.printf("T=%.2f degC, Humidity=%.2f %\n", hdc1080_result.temperature, hdc1080_result.humidity);
  // Set the first element of appData as the battery level
  appData[0] = GetBatteryLevel();

  // // Copy hdc1080 data to appData starting from the second element
  memcpy(&appData[1], &hdc1080_result, sizeof(hdc1080_data));

  // // Copy bmp280 data to appData after hdc1080 data
  memcpy(&appData[sizeof(hdc1080_data) + 1], &bmp280_result, sizeof(bmp280_data));

  // // Calculate the size of appData
  // Serial.print("appDataSize:");
  // Serial.println(appDataSize);
  // // Uncomment the code below to print the values in appData
  // for (int i = 0; i < appDataSize; i++)
  // {
  //   Serial.print("Byte ");
  //   Serial.print(i);
  //   Serial.print(": 0x");
  //   Serial.println(appData[i], HEX);
  // }
  Serial.flush();
}

void Tx_setup()
{
  Serial.begin(115200);
  Mcu.begin();
  print_wakeup_reason();
  txNumber = 0;

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  // set sleep mode
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  Sleep_uSec += randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
  // setting wakeup time
  esp_sleep_enable_timer_wakeup(Sleep_uSec); // uSecond

  bootCount++;
  Serial.printf("this is boot number %d\n", bootCount);
}

void Tx_loop()
{
  if (lora_idle == true)
  {
    prepareTxFrame(0);
    Radio.Send((uint8_t *)appData, appDataSize); // send the package out
    lora_idle = false;
  }
  Radio.IrqProcess();
}

void OnTxDone(void)
{
  Serial.println("TX done......");
  lora_idle = true;
  Radio.Sleep();
  // enter deep sleep
  esp_deep_sleep_start();
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  Serial.println("TX Timeout......");
  lora_idle = true;
}
int16_t rssi, rxSize;
void Rx_setup()
{
  Serial.begin(115200);
  Mcu.begin();

  txNumber = 0;
  rssi = 0;

  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}

void Rx_loop()
{
  if (lora_idle)
  {
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0);
  }
  Radio.IrqProcess();
}
void convertPacket();
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  rssi = rssi;
  rxSize = size;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  convertPacket();
  Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, rxSize);
  lora_idle = true;
}

void convertPacket()
{
  // Uncomment the code below to print the values in rxpacket
  for (int i = 0; i < rxSize; i++)
  {
    Serial.print("Byte ");
    Serial.print(i);
    Serial.print(": 0x");
    Serial.println(rxpacket[i], HEX);
  }

  // convert battery level
  uint8_t batLevel = rxpacket[0];
  Serial.print("battery level:");
  Serial.println(batLevel);
  // convert hdc1080 temperature
  float temperature = *(float *)(rxpacket + 1);
  Serial.print("HDC1080 temperature:");
  Serial.println(temperature);
  // convert hdc1080 humidity
  float humidity = *(float *)(rxpacket + 5);
  Serial.print("HDC1080 humidity:");
  Serial.println(humidity);
  Serial.println(" p.s. if HDC1080 humidity is going to 0 suddenly, pls use hdc1080 internal heater function to reduce the effects of condensation on the sensor's surface.");
  // convert bmp280 temperature
  float bmpTemperature = *(float *)(rxpacket + 9);
  Serial.print("bmp280 internal temperature:");
  Serial.println(bmpTemperature);
  Serial.println(" p.s. bmp280 internal temperature should be higher than ambient temperature");
  // convert bmp280 pressure
  float bmpPressure = *(float *)(rxpacket + 13);
  Serial.print("bmp pressure:");
  Serial.println(bmpPressure);
}

void setup()
{
#ifdef DEVICE_AS_RX
  Rx_setup();
#else
  Tx_setup();
#endif
}
void loop()
{
#ifdef DEVICE_AS_RX
  Rx_loop();
#else
  Tx_loop();
#endif
}