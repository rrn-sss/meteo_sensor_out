// Определение ESP32_C6 передаётся через build_flags в platformio.ini для [env:xiao_esp32c6]

#define SHTC3_SENSOR_ENABLED // Раскомментировать для использования SHTC3 вместо BME280
                             // #define SHT31_SENSOR_ENABLED // Раскомментировать для использования SHT31 вместо BME280
#define TEST_W_SERIAL        // Раскомментировать для отладки через Serial
#define EINK_DISPLAY_ENABLED // Раскомментировать для включения e-ink дисплея

#ifdef SHTC3_SENSOR_ENABLED
#include <Adafruit_SHTC3.h>
#elif defined(SHT31_SENSOR_ENABLED)
#include <Adafruit_SHT31.h>
#else
#include <Adafruit_BME280.h>
#endif
#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_sleep.h>

#ifdef EINK_DISPLAY_ENABLED
#include <GxEPD2_BW.h>
#endif

#ifdef ESP32_C6
// Пины и адреса I2C
// Синхронизировано с распиновкой XIAO ESP32-C6 (SDA на GPIO22 по схеме)
#define SDA_PIN 22 // SDA для XIAO ESP32-C6
#define SCL_PIN 23 // SCL для XIAO ESP32-C6

// Пины для SPI (стандартные для XIAO ESP32-C6)
#define SPI_SCK_PIN 19  // Пин SCK для SPI
#define SPI_MISO_PIN 20 // Пин MISO для SPI
#define SPI_MOSI_PIN 18 // Пин MOSI для SPI
#define NRF24_CE_PIN 0  // Пин CE для nRF24L01+
#define NRF24_CS_PIN 21 // Пин CS для SPI (лучше привязывать к SPI SS)

// Пины для измерения заряда батареи
#define BAT_ADC_PIN 2     // GPIO2  — вход АЦП (средняя точка делителя R1/R2)
#define BAT_MOSFET_PIN 17 // GPIO17 — затвор 2N7000 (включает делитель)
// Пины для e-ink дисплея (SSD168, 200x200)
#define EINK_CS_PIN 1   // D1 - Chip Select
#define EINK_DC_PIN 16  // D6 - Data/Command
#define EINK_BUSY_PIN 2 // D2 - Busy (shared с BAT_ADC_PIN!)

// GxEPD2 экземпляр для 200x200 SSD168 (SSD1681) монохромного дисплея
#ifdef EINK_DISPLAY_ENABLED
GxEPD2_BW<GxEPD2_154_GDEY0154D67, GxEPD2_154_GDEY0154D67::HEIGHT> display(GxEPD2_154_GDEY0154D67(EINK_CS_PIN, EINK_DC_PIN, /*RST=*/-1, EINK_BUSY_PIN));
#endif

#else
// Пины и адреса I2C
#define SDA_PIN 8           // Пины для I2C
#define SCL_PIN 9           // Пины для I2C

// Пины для SPI
#define SPI_SCK_PIN 4       // Пин SCK для SPI
#define SPI_MISO_PIN 5      // Пин MISO для SPI
#define SPI_MOSI_PIN 6      // Пин MOSI для SPI
#define NRF24_CE_PIN 7      // Пин CE для nRF24L01+
#define NRF24_CS_PIN 10     // Пин CS для SPI
// Пины для измерения заряда батареи
#define BAT_ADC_PIN TODO    // GPIO2  — вход АЦП (средняя точка делителя R1/R2)
#define BAT_MOSFET_PIN TODO // GPIO17 — затвор 2N7000 (включает делитель)
#endif                      // ESP32_C6

#ifdef SHTC3_SENSOR_ENABLED
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3(); // Объект для работы с SHTC3
#elif defined(SHT31_SENSOR_ENABLED)
Adafruit_SHT31 sht31 = Adafruit_SHT31(); // Объект для работы с SHT31
#else
Adafruit_BME280 bme280; // Объект для работы с BME280
#endif

const uint64_t NRF24_ADDRESS = 0xA337D135B1ULL; // Адрес получателя для nRF24L01+
RF24 radio(NRF24_CE_PIN, NRF24_CS_PIN);         // Объект для работы с nRF24L01+

// Структура данных для отправки по радио
typedef struct __attribute__((packed))
{
  float temperature{0.0f}; // температура в градусах Цельсия
  float humidity{0.0f};    // относительная влажность в процентах
  uint16_t pressure{0};    // давление в гПа
  uint16_t bat_charge{0};  // заряд батареи в процентах
} OutSensorData_t;

float temp_val, pressure_val, humidity_val; // Глобальные переменные для хранения данных сенсоров

// ---------------------------------------------------------------------------
// Измерение заряда аккумулятора 18650
// Схема: V_bat → R1(280кОм) → GPIO2(ADC) → R2(280кОм) → Drain(2N7000) → GND
//        Gate(2N7000) → GPIO17
// Делитель 1:2 → V_adc = V_bat / 2
// 18650: 4200 мВ = 100 %, 3200 мВ = 0 %
// ---------------------------------------------------------------------------
uint16_t readBatCharge(bool f_debug = false)
{
  pinMode(BAT_MOSFET_PIN, OUTPUT);
  digitalWrite(BAT_MOSFET_PIN, HIGH); // включаем делитель через MOSFET
  delay(10);                          // ждём стабилизации (RC-цепь 280кОм × C_ввода)

  // Некоторые Arduino-ядра ESP32 не предоставляют adcAttachPin(),
  // поэтому выполняем одно чтение, чтобы pin зарегистрировался в ADC-подсистеме,
  // затем устанавливаем аттенюацию для корректного диапазона.
  analogRead(BAT_ADC_PIN);                        // принудительно зарегистрировать пин
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db); // диапазон 0–3.3 В

  // Усреднение 8 выборок для снижения шума АЦП
  uint32_t adcSum = 0;
  for (int i = 0; i < 8; i++)
  {
    adcSum += analogReadMilliVolts(BAT_ADC_PIN);
    delayMicroseconds(500);
  }
  uint32_t adcMv = adcSum / 8;

  digitalWrite(BAT_MOSFET_PIN, LOW); // выключаем делитель — экономим энергию

  uint32_t batMv = adcMv * 2; // восстанавливаем реальное напряжение (делитель 1:2)

#ifdef TEST_W_SERIAL
  if (f_debug)
  {
    Serial.print("DEBUG: ADC mV=");
    Serial.print(adcMv);
    Serial.print(" bat mV=");
    Serial.println(batMv);
  }
#endif

  if (batMv >= 4200)
    return 100;
  if (batMv <= 3200)
    return 0;
  return (uint16_t)((batMv - 3200UL) * 100UL / 1000UL);
}

// При ошибке инициализации — сразу в deep sleep, не тратим батарею
void goToSleep()
{
#ifdef TEST_W_SERIAL
  Serial.println("DEBUG: Going to sleep...");
  Serial.flush(); // Дождаться вывода перед отключением периферии
  delay(5000);
  esp_restart(); // Перезагрузка вместо deep sleep для отладки через Serial
#else
  esp_sleep_enable_timer_wakeup(1ULL * 60 * 1000000); // 1 минута
  esp_deep_sleep_start();
#endif
}

// ---------------------------------------------------------------------------
// Обновление e-ink дисплея (SSD168, 200x200) с данными датчиков
// ВАЖНО: вызывать ПОСЛЕ измерения батареи, т.к. BUSY пин (GPIO2) используется для обоих
// ---------------------------------------------------------------------------
#ifdef EINK_DISPLAY_ENABLED
void updateDisplay(const OutSensorData_t &sensorData)
{
#ifdef TEST_W_SERIAL
  Serial.println("DEBUG: Initializing e-ink display...");
#endif

  // Переинициализация SPI для дисплея (NRF24 уже выключен)
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  // Устанавливаем пины CS и DC как выходы (SPI.begin не делает это автоматически)
  pinMode(EINK_CS_PIN, OUTPUT);
  pinMode(EINK_DC_PIN, OUTPUT);

  display.init(/*115200=*/false);
  display.setRotation(1);
  display.fillScreen(GxEPD_WHITE);

  // Отображение данных
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(10, 30);

  // Температура
  display.print("Temp: ");
  display.print(sensorData.temperature, 1);
  display.println(" C");

  // Влажность
  display.print("Humidity: ");
  display.print(sensorData.humidity, 1);
  display.println(" %");

  // Давление
  display.print("Pressure: ");
  display.print(sensorData.pressure);
  display.println(" hPa");

  // Заряд батареи
  display.print("Battery: ");
  display.print(sensorData.bat_charge);
  display.println(" %");

#ifdef TEST_W_SERIAL
  Serial.println("DEBUG: Updating e-ink display...");
#endif

  // Полное обновление (full refresh) для корректного отображения
  display.display(false);

  // Выключаем дисплей для экономии энергии
  display.powerOff();

#ifdef TEST_W_SERIAL
  Serial.println("DEBUG: E-ink display update complete");
#endif
  SPI.end();
}
#endif // EINK_DISPLAY_ENABLED

void setup()
{
  setCpuFrequencyMhz(80); // Сразу снижаем частоту CPU для экономии энергии

  // Выключаем WiFi для экономии энергии
  WiFi.mode(WIFI_OFF);

#ifdef TEST_W_SERIAL
  Serial.begin(115200);
  Serial.println("DEBUG: CPU freq set to 80 MHz");
  delay(5000); // Ждем стабилизации Serial
#endif
  Wire.begin(SDA_PIN, SCL_PIN); // Инициализация I2C с заданными пинами
  Wire.setClock(400000);        // 400 kHz — быстрее считывание, меньше активного времени
  // I2C initialized

#ifdef SHTC3_SENSOR_ENABLED
  if (!shtc3.begin())
  {
#ifdef TEST_W_SERIAL
    Serial.println("Couldn't find SHTC3");
#endif
    goToSleep();
  }

#ifdef TEST_W_SERIAL
  Serial.println("Found SHTC3 sensor");
#endif

  shtc3.sleep(false); // Выводим SHTC3 из сна
  // delay(5);           // Wake-up time SHTC3 ~240 мкс + время стабилизации
  sensors_event_t humidity, temp;
  shtc3.getEvent(&humidity, &temp);
  temp_val = temp.temperature;
  humidity_val = humidity.relative_humidity;
  pressure_val = 0; // SHTC3 не измеряет давление
  shtc3.sleep(true);
#elif defined(SHT31_SENSOR_ENABLED)
  if (!sht31.begin(0x44)) // Адрес 0x44 или 0x45
  {
#ifdef TEST_W_SERIAL
    Serial.println("Couldn't find SHT31");
#endif
    goToSleep();
  }

#ifdef TEST_W_SERIAL
  Serial.println("Found SHT31 sensor");
#endif

  temp_val = sht31.readTemperature();
  humidity_val = sht31.readHumidity();
  pressure_val = 0; // SHT31 не измеряет давление
#else
  if (!bme280.begin(0x76)) // Адрес 0x76 или 0x77
  {
#ifdef TEST_W_SERIAL
    Serial.println("Couldn't find BME280 sensor");
#endif
    goToSleep();
  }

#ifdef TEST_W_SERIAL
  Serial.println("Found BME280 sensor");
#endif

  // setSampling ВСЕГДА — не только в debug! Иначе BME280 в normal mode потребляет ~3.6 мА
  bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                     Adafruit_BME280::SAMPLING_X1, // temperature
                     Adafruit_BME280::SAMPLING_X1, // pressure
                     Adafruit_BME280::SAMPLING_X1, // humidity
                     Adafruit_BME280::FILTER_OFF);

  bme280.takeForcedMeasurement();
  temp_val = bme280.readTemperature();
  pressure_val = bme280.readPressure() / 100.0F; // hPa
  humidity_val = bme280.readHumidity();
#endif // SHTC3_SENSOR_ENABLED

#ifdef TEST_W_SERIAL
  Serial.print("DEBUG: Readings — Temp: ");
  Serial.print(temp_val);
  Serial.print(" *C, Humidity: ");
  Serial.print(humidity_val);
  Serial.print(" %, Pressure: ");
  Serial.print(pressure_val);
  Serial.print(" hPa, charge: ");
  uint16_t batCharge = readBatCharge();
  Serial.print(batCharge);
  Serial.println(" %");
#endif

  OutSensorData_t data;
  data.temperature = temp_val;
  data.humidity = humidity_val;
  data.pressure = static_cast<uint16_t>(pressure_val);
  data.bat_charge = readBatCharge(true);

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  // Setup and configure rf radio
  if (!radio.begin())
  {
#ifdef TEST_W_SERIAL
    Serial.println(F("radio hardware not responding!"));
#endif
    goToSleep(); // Сон вместо delay(5000) + restart
  }

  delayMicroseconds(1500);              // Ждём стабилизации модуля после включения (Tpd2stby = 1.5ms)
  radio.setPALevel(RF24_PA_HIGH);       // Уровень мощности передачи
  radio.setDataRate(RF24_250KBPS);      // Скорость передачи данных
  radio.setRetries(5, 3);               // 3 попытки с интервалом 5 (5*250 мкс) - более надежно
  radio.setChannel(103);                // Канал 103 (2503 MHz)
  radio.openWritingPipe(NRF24_ADDRESS); // Адрес получателя
  radio.stopListening();                // Переходим в режим передачи
  delayMicroseconds(150);               // Ждём переключения режима TX (130 мкс по datasheet)

#ifdef TEST_W_SERIAL
  Serial.println(F("RF24 radio initialized."));
#endif

  radio.flush_tx(); // Очищаем TX буфер перед отправкой
  bool ok = radio.write(&data, sizeof(OutSensorData_t));
#ifdef TEST_W_SERIAL
  Serial.println(ok ? "DEBUG: Data sent OK" : "DEBUG: Data send FAILED");
  radio.printDetails();
  Serial.flush(); // Дождаться вывода перед отключением периферии
#endif
  radio.powerDown();
  SPI.end();
  Wire.end();

#ifdef EINK_DISPLAY_ENABLED
  updateDisplay(data);
#endif

  // GPIO изолировать НЕ НУЖНО — ESP32-C6 делает это автоматически при deep sleep
  goToSleep();
}

void loop()
{
#ifdef TEST_W_SERIAL
  setup(); // Для отладки через Serial — выполняем setup() в loop(), чтобы не уходить в сон
#endif
}
