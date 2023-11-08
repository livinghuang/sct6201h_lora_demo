#include "sct62_bsp.h"
#include "Wire.h"
#include "BMP280.h"
#include "HDC1080.h"
#include "LoRaWan_APP.h"
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
  // Serial.println("I2C ON");
  delay(15);
  digitalWrite(pVext, LOW);
  // Serial.println("Vext ON");
}

void power_Off_Sensor_Bus()
{
  pinMode(pVext, OUTPUT);
  // according the I2C bus collision with the JTAG/USB pin , it is internal pull down, we need to set it low before turn off the Vext to keep the Vext level in low.
  pinMode(pSDA, OUTPUT);
  pinMode(pSCL, OUTPUT);
  digitalWrite(pSDA, LOW);
  digitalWrite(pSCL, LOW);
  // Serial.println("I2C OFF");
  delay(15);
  digitalWrite(pVext, HIGH);
  // Serial.println("Vext OFF");
}

/* OTAA para*/
uint8_t devEui[] = {0x22, 0x32, 0x33, 0x00, 0x00, 0x88, 0x88, 0x08};
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appKey[] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88};

/* ABP para*/
uint8_t nwkSKey[] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88};
uint8_t appSKey[] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88};
uint32_t devAddr = (uint32_t)0x88888888;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_AS923_AS2;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 120000;

/*OTAA or ABP*/
bool overTheAirActivation = false;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;
/*!
 * Number of trials to transmit the frame, if the LoRaMAC layer did not
 * receive an acknowledgment. The MAC performs a datarate adaptation,
 * according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
 * to the following table:
 *
 * Transmission nb | Data Rate
 * ----------------|-----------
 * 1 (first)       | DR
 * 2               | DR
 * 3               | max(DR-1,0)
 * 4               | max(DR-1,0)
 * 5               | max(DR-2,0)
 * 6               | max(DR-2,0)
 * 7               | max(DR-3,0)
 * 8               | max(DR-3,0)
 *
 * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
 * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
 */
uint8_t confirmedNbTrials = 4;

void fetchSensorData()
{
  int i;
  for (i = 0; i < 5; i++)
  {
    power_On_Sensor_Bus();
    delay(15);
    hdc1080_fetch();
    delay(10);
    power_Off_Sensor_Bus();
    delay(15);
  }

  for (i = 0; i < 5; i++)
  {
    power_On_Sensor_Bus();
    delay(10);
    bmp280_fetch();
    // delay(10);
    power_Off_Sensor_Bus();
    delay(5);
  }
}

// Prepares the payload of the frame
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

// if true, next uplink will add MOTE_MAC_DEVICE_TIME_REQ

void setup()
{
  Serial.begin(115200);
  Mcu.begin();

  deviceState = DEVICE_STATE_INIT;
}

void loop()
{
  switch (deviceState)
  {
  case DEVICE_STATE_INIT:
  {
#if (LORAWAN_DEVEUI_AUTO)
    LoRaWAN.generateDeveuiByChipID();
#endif
    LoRaWAN.init(loraWanClass, loraWanRegion);
    break;
  }
  case DEVICE_STATE_JOIN:
  {
    LoRaWAN.join();
    break;
  }
  case DEVICE_STATE_SEND:
  {
    prepareTxFrame(appPort);
    LoRaWAN.send();
    deviceState = DEVICE_STATE_CYCLE;
    break;
  }
  case DEVICE_STATE_CYCLE:
  {
    // Schedule next packet transmission
    txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
    deviceState = DEVICE_STATE_SLEEP;
    break;
  }
  case DEVICE_STATE_SLEEP:
  {
    LoRaWAN.sleep(loraWanClass);
    break;
  }
  default:
  {
    deviceState = DEVICE_STATE_INIT;
    break;
  }
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
