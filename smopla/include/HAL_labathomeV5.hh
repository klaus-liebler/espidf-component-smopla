#pragma once

#include "HAL.hh"

#include <inttypes.h>
#include <limits>
#include <algorithm>
#include <common.hh>
#include <common-esp32.hh>

#include <driver/mcpwm.h>
#include <driver/ledc.h>
#include <driver/adc.h>
#include <driver/i2c.h>
#include <driver/rmt.h>

#include <errorcodes.hh>
#include <rgbled.hh>
#include <bh1750.hh>
#include <bme280.hh>
#include <ads1115.hh>
#include <ccs811.hh>
#include <hdc1080.hh>
#include <aht_sensor.hh>
#include <owb.h>
#include <owb_rmt.h>
#include <ds18b20.h>
#include <rotenc.hh>

#include <driver/i2s.h>
#include <MP3Player.hh>

#include "winfactboris_messages.hh"

FLASH_FILE(alarm_co2_mp3)
FLASH_FILE(alarm_temperature_mp3)
FLASH_FILE(nok_mp3)
FLASH_FILE(ok_mp3)
FLASH_FILE(ready_mp3)
FLASH_FILE(fanfare_mp3)
FLASH_FILE(negative_mp3)
FLASH_FILE(positive_mp3)
FLASH_FILE(siren_mp3)
const uint8_t *SOUNDS[] = {nullptr, alarm_co2_mp3_start, alarm_temperature_mp3_start, nok_mp3_start, ok_mp3_start, ready_mp3_start, fanfare_mp3_start, negative_mp3_start, positive_mp3_start, siren_mp3_start};
const size_t SONGS_LEN[] = {0, alarm_co2_mp3_size, alarm_temperature_mp3_size, nok_mp3_size, ok_mp3_size, ready_mp3_size, fanfare_mp3_size, negative_mp3_size, positive_mp3_size, siren_mp3_size};
MP3::Player mp3player;

typedef gpio_num_t Pintype;

constexpr Pintype PIN_R3_1 = (Pintype)36;
constexpr Pintype PIN_MOVEMENT = (Pintype)39;
constexpr Pintype PIN_SW = (Pintype)34;
constexpr adc1_channel_t CHANNEL_SWITCHES = ADC1_CHANNEL_6;
constexpr Pintype PIN_ROTENC_A = (Pintype)35;
constexpr Pintype PIN_FAN2_DRIVE = (Pintype)32;
constexpr Pintype PIN_FAN1_SENSE = (Pintype)33;
constexpr Pintype PIN_SERVO2 = (Pintype)33;
constexpr Pintype PIN_SPEAKER = (Pintype)25;
constexpr Pintype PIN_LED_STRIP = (Pintype)26;

constexpr Pintype PIN_MULTI1 = (Pintype)27;
constexpr Pintype PIN_ONEWIRE = (Pintype)14;
constexpr Pintype PIN_FAN1_DRIVE = (Pintype)12;
constexpr Pintype PIN_LED_POWER_WHITE = (Pintype)13;

constexpr Pintype PIN_SPI_MOSI = (Pintype)23;
constexpr Pintype PIN_I2C_SDA = (Pintype)22;
constexpr Pintype PIN_I2C_SCL = (Pintype)21;
constexpr Pintype PIN_SPI_MISO = (Pintype)19;
constexpr Pintype PIN_SPI_CLK = (Pintype)18;
constexpr Pintype PIN_SPI_IO1 = (Pintype)5;
constexpr Pintype PIN_HEATER = (Pintype)17;
constexpr Pintype PIN_MULTI3 = (Pintype)16;
constexpr Pintype PIN_MULTI2 = (Pintype)4;
constexpr Pintype PIN_SPI_IO2 = (Pintype)0;
constexpr Pintype PIN_R3_ON = (Pintype)2;
constexpr Pintype PIN_ROTENC_B = (Pintype)15;

constexpr Pintype PIN_SERVO1 = PIN_MULTI1;
constexpr Pintype PIN_I2S_SCK = PIN_MULTI1;
// constexpr Pintype PIN_485_DE = PIN_MULTI1;
// constexpr Pintype PIN_EXT1 = PIN_MULTI1;

// constexpr Pintype PIN_485_RO = PIN_MULTI2;
// constexpr Pintype PIN_CAN_TX = PIN_MULTI2;
// constexpr Pintype PIN_EXT2 = PIN_MULTI2;
constexpr Pintype PIN_I2S_WS = PIN_MULTI2;

// constexpr Pintype PIN_485_DI = PIN_MULTI3;
// constexpr Pintype PIN_CAN_RX = PIN_MULTI3;
constexpr Pintype PIN_I2S_SD = PIN_MULTI3;
// constexpr Pintype PIN_EXT3 = PIN_MULTI3;

enum class MODE_IO33
{
    SERVO2,
    FAN1_SENSE,
};

enum class MODE_MULTI1_PIN
{
    I2S,
    RS485,
    SERVO1,
    EXT,
};

enum class MODE_MULTI_2_3_PINS
{
    I2S,
    RS485,
    CAN,
    EXT,
};

enum class Button : uint8_t
{
    BUT_GREEN = 1,
    BUT_ENCODER = 0,
    BUT_RED = 2,
};

constexpr size_t ANALOG_INPUTS_LEN{0};
constexpr size_t LED_NUMBER{8};
constexpr rmt_channel_t CHANNEL_ONEWIRE_TX{RMT_CHANNEL_1};
constexpr rmt_channel_t CHANNEL_ONEWIRE_RX{RMT_CHANNEL_2};
constexpr i2c_port_t I2C_PORT{I2C_NUM_1};
constexpr uint32_t DEFAULT_VREF{1100}; // Use adc2_vref_to_gpio() to obtain a better estimate
constexpr uint16_t sw_limits[]{160, 480, 1175, 1762, 2346, 2779, 3202};
constexpr float SERVO_MIN_PULSEWIDTH{500.0};  // Minimum pulse width in microsecond
constexpr float SERVO_MAX_PULSEWIDTH{2400.0}; // Maximum pulse width in microsecond
constexpr float SERVO_MAX_DEGREE{180.0};      // Maximum angle in degree upto which servo can rotate

constexpr ledc_timer_bit_t power_ledc_timer_duty_resolution{LEDC_TIMER_10_BIT};

constexpr mcpwm_timer_t MCPWM_TIMER_SERVO{MCPWM_TIMER_0};
constexpr mcpwm_io_signals_t MCPWM_IO_SERVO1{MCPWM0A};
constexpr mcpwm_generator_t MCPWM_GEN_SERVO1{MCPWM_GEN_A};
constexpr mcpwm_io_signals_t MCPWM_IO_SERVO2{MCPWM0B};
constexpr mcpwm_generator_t MCPWM_GEN_SERVO2{MCPWM_GEN_B};

constexpr mcpwm_timer_t MCPWM_TIMER_FAN{MCPWM_TIMER_1};
constexpr mcpwm_io_signals_t MCPWM_IO_FAN1{MCPWM1A};
constexpr mcpwm_generator_t MCPWM_GEN_FAN1{MCPWM_GEN_A};
constexpr mcpwm_io_signals_t MCPWM_IO_FAN2{MCPWM1B};
constexpr mcpwm_generator_t MCPWM_GEN_FAN2{MCPWM_GEN_B};

constexpr mcpwm_timer_t MCPWM_TIMER_HEATER_OR_LED_POWER{MCPWM_TIMER_2};
constexpr mcpwm_io_signals_t MCPWM_IO_HEATER_OR_LED_POWER{MCPWM2A};
constexpr mcpwm_generator_t MCPWM_GEN_HEATER_OR_LED_POWER{MCPWM_GEN_A};

constexpr int FREQUENCY_HEATER{1}; // see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html#resolution
constexpr int FREQUENCY_SERVO{50};
constexpr int FREQUENCY_FAN{100};
constexpr int FREQUENCY_LED{300};

constexpr bool NS4168_ACTIVE{false};
constexpr bool MD8002_ACTIVE{!NS4168_ACTIVE};

extern "C" void sensorTask(void *pvParameters);

class HAL_labathome : public HAL
{
private:
    MODE_IO33 mode_io33;
    MODE_MULTI1_PIN mode_multi1;
    MODE_MULTI_2_3_PINS mode_multi23;

    esp_adc_cal_characteristics_t *adc_chars{nullptr};

    // management objects
    RGBLED<LED_NUMBER, DeviceType::WS2812> *strip{nullptr};
    cRotaryEncoder *rotenc{nullptr};
    CCS811::M *ccs811dev{nullptr};

    // SensorValues
    uint32_t buttonState{0}; // see Button-Enum for meaning of bits
    // int rotaryDetents rotary-Encoder Value in Object
    bool movementIsDetected{false};
    uint16_t voltage_USB_PD{0};
    float ambientBrightnessLux_analog{std::numeric_limits<float>::quiet_NaN()};
    float ambientBrightnessLux_digital{std::numeric_limits<float>::quiet_NaN()};
    float heaterTemperatureDegCel{std::numeric_limits<float>::quiet_NaN()};
    float airTemperatureDegCel{std::numeric_limits<float>::quiet_NaN()};
    float airPressurePa{std::numeric_limits<float>::quiet_NaN()};
    float airRelHumidityPercent{std::numeric_limits<float>::quiet_NaN()};
    float airCo2PPM{std::numeric_limits<float>::quiet_NaN()};
    float airQualityPercent{std::numeric_limits<float>::quiet_NaN()};
    float airSpeedMeterPerSecond{std::numeric_limits<float>::quiet_NaN()};
    float wifiRssiDb{std::numeric_limits<float>::quiet_NaN()};
    // float analogInputVolt{std::numeric_limits<float>::quiet_NaN()};
    uint32_t sound{0};
    float fan1RotationsRpM{std::numeric_limits<float>::quiet_NaN()};

    bool heaterEmergencyShutdown{false};

    float AnalogInputs[ANALOG_INPUTS_LEN] = {};

    uint16_t ds4525doPressure;

    void MP3Loop()
    {
        // sizeof(MP3::Player);64byte
        if (MD8002_ACTIVE)
        {
            mp3player.InitInternalDACMonoRightPin25();
        }
        else if (NS4168_ACTIVE)
        {
            //mp3player.InitExternalI2SDAC(I2S_PORT_LOUDSPEAKER, PIN_NS4168_SCK, PIN_NS4168_WS, PIN_NS4168_SDAT);
        }

        while (true)
        {
            mp3player.Loop();
        }
    }

    void readBinaryAndAnalogIOs()
    {

        int adc_reading = adc1_get_raw(CHANNEL_SWITCHES);
        // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        // printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        int i = 0;
        for (i = 0; i < sizeof(sw_limits) / sizeof(uint16_t); i++)
        {
            if (adc_reading < sw_limits[i])
                break;
        }
        i = ~i;
        this->buttonState = i;

        // wifi_ap_record_t ap_info;
        // esp_wifi_sta_get_ap_info(&ap_info);//TODO may only be called, when ESP32 is connected...
        // this->wifiRssiDb=ap_info.rssi;

        // Es wird nicht unterschieden, ob der eingang "nur" zum Messen von analogen Spannung verwendet wird oder ob es sich um den Trigger-Eingang des Zeitrelais handelt. Im zweiten Fall muss einfach eine Zeitrelais-Schaltung basierend auf der Grenzüberschreitung des Analogen Messwertes in der Funktionsblock-Spache realisiert werden
        int analogIn = 0;//adc1_get_raw(CHANNEL_ANALOGIN_OR_ROTB);
        this->AnalogInputs[0] = 11 * esp_adc_cal_raw_to_voltage(analogIn, adc_chars); // die Multiplikation mit 11 wegen dem Spannungsteiler
        int ldrIn = 0;//adc1_get_raw(CHANNEL_LDR_OR_ROTA);
        this->ambientBrightnessLux_analog = esp_adc_cal_raw_to_voltage(ldrIn, adc_chars); // TODO Messkurce erstellen
        this->movementIsDetected = gpio_get_level(PIN_MOVEMENT);    
    }

    void SensorLoop()
    {
        int64_t nextOneWireReadout{INT64_MAX};
        int64_t nextBME280Readout{INT64_MAX};
        int64_t nextBH1750Readout{INT64_MAX};
        int64_t nextBinaryAndAnalogReadout{0};

        uint32_t oneWireReadoutIntervalMs{800}; // 10bit -->187ms Conversion time, 12bit--> 750ms
        uint32_t bme280ReadoutIntervalMs{UINT32_MAX};
        uint32_t bh1750ReadoutIntervalMs{200};

        // OneWire
        DS18B20_Info *ds18b20_info = NULL;
        owb_rmt_driver_info rmt_driver_info;
        OneWireBus *owb = owb_rmt_initialize(&rmt_driver_info, PIN_ONEWIRE, CHANNEL_ONEWIRE_TX, CHANNEL_ONEWIRE_RX);
        owb_use_crc(owb, true); // enable CRC check for ROM code
        // Find all connected devices
        OneWireBus_ROMCode rom_code;
        owb_status status = owb_read_rom(owb, &rom_code);
        if (status == OWB_STATUS_OK)
        {
            ds18b20_info = ds18b20_malloc();      // heap allocation
            ds18b20_init_solo(ds18b20_info, owb); // only one device on bus
            ds18b20_use_crc(ds18b20_info, true);  // enable CRC check on all reads
            ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION_12_BIT);
            ds18b20_convert_all(owb);
            nextOneWireReadout = GetMillis() + oneWireReadoutIntervalMs;
            ESP_LOGI(TAG, "OneWire: DS18B20 successfully initialized.");
        }
        else
        {
            ESP_LOGE(TAG, "OneWire: An error occurred reading ROM code: %d", status);
        }

        // BME280
        BME280 *bme280 = new BME280(I2C_PORT, BME280_ADRESS::PRIM);
        if (bme280->Init(&bme280ReadoutIntervalMs) == ESP_OK)
        {
            bme280->TriggerNextMeasurement();
            bme280ReadoutIntervalMs += 20;
            bme280->TriggerNextMeasurement();
            nextBME280Readout = GetMillis() + bme280ReadoutIntervalMs;
            ESP_LOGI(TAG, "I2C: BME280 successfully initialized.");
        }
        else
        {
            ESP_LOGW(TAG, "I2C: BME280 not found");
        }

        // BH1750
        BH1750 *bh1750 = new BH1750(I2C_PORT, BH1750_ADRESS::LOW);
        if (bh1750->Init(BH1750_OPERATIONMODE::CONTINU_H_RESOLUTION) == ESP_OK)
        {
            bh1750ReadoutIntervalMs = 200;
            nextBH1750Readout = GetMillis() + bh1750ReadoutIntervalMs;
            ESP_LOGI(TAG, "I2C: BH1750 successfully initialized.");
        }
        else
        {
            ESP_LOGW(TAG, "I2C: BH1750 not found");
        }

        // CCS811
        ccs811dev = new CCS811::M(I2C_PORT, CCS811::ADDRESS::ADDR0, CCS811::MODE::_1SEC, (gpio_num_t)GPIO_NUM_NC);

        while (true)
        {
            if (GetMillis64() > nextBinaryAndAnalogReadout)
            {
                readBinaryAndAnalogIOs();
                nextBinaryAndAnalogReadout = GetMillis64() + 100;
                if (!mp3player.IsEmittingSamples())
                {
                    this->sound = 0;
                }
            }

            if (GetMillis64() > nextOneWireReadout)
            {
                ds18b20_read_temp(ds18b20_info, &(this->heaterTemperatureDegCel));
                ds18b20_convert_all(owb);
                nextOneWireReadout = GetMillis64() + oneWireReadoutIntervalMs;
            }
            if (GetMillis64() > nextBME280Readout)
            {
                bme280->GetDataAndTriggerNextMeasurement(&this->airTemperatureDegCel, &this->airPressurePa, &this->airRelHumidityPercent);
                nextBME280Readout = GetMillis64() + bme280ReadoutIntervalMs;
            }
            if (GetMillis64() > nextBH1750Readout)
            {
                bh1750->Read(&(this->ambientBrightnessLux_digital));
                nextBH1750Readout = GetMillis64() + bh1750ReadoutIntervalMs;
            }

            
            ccs811dev->Loop(GetMillis64());
   

            if (ccs811dev->HasValidData())
            {
                this->airCo2PPM = ccs811dev->Get_eCO2();
            }
           
            vTaskDelay(1);
        }
    }

public:
    HAL_labathome(MODE_IO33 mode_io33, MODE_MULTI1_PIN mode_multi1, MODE_MULTI_2_3_PINS mode_multi23) : mode_io33(mode_io33), mode_multi1(mode_multi1), mode_multi23(mode_multi23)
    {
    }

    ErrorCode OutputOneLineStatus() override
    {
        uint32_t heap = esp_get_free_heap_size();
        bool red = GetButtonRedIsPressed();
        bool yel = GetButtonEncoderIsPressed();
        bool grn = GetButtonGreenIsPressed();
        bool mov = IsMovementDetected();
        float htrTemp{0.f};
        int enc{0};
        GetEncoderValue(&enc);
        int32_t sound{0};
        GetSound(&sound);
        float spply = GetUSBCVoltage();

        float bright{0.0};
        GetAmbientBrightness(&bright);

        float co2{0};
        GetHeaterTemperature(&htrTemp);
        float airTemp{0.f};
        GetAirTemperature(&airTemp);
        float airPres{0.f};
        GetAirPressure(&airPres);
        float airHumid{0.f};
        GetAirRelHumidity(&airHumid);
        GetCO2PPM(&co2);
        float *analogVolt{nullptr};
        GetAnalogInputs(&analogVolt);
        ESP_LOGI(TAG, "Heap %6d  RED %d YEL %d GRN %d MOV %d ENC %d SOUND %d SUPPLY %4.1f BRGHT %4.1f HEAT %4.1f AIRT %4.1f AIRPRS %5.0f AIRHUM %3.0f CO2 %5.0f, ANALOGIN %4.1f",
                 heap, red, yel, grn, mov, enc, sound, spply, bright, htrTemp, airTemp, airPres, airHumid, co2, analogVolt[0]);
        return ErrorCode::OK;
    }

    ErrorCode HardwareTest() override
    {
        return ErrorCode::OK;
    }

    int64_t IRAM_ATTR GetMicros()
    {
        return esp_timer_get_time();
    }

    uint32_t GetMillis()
    {
        return (uint32_t)(esp_timer_get_time() / 1000ULL);
    }

    int64_t GetMillis64()
    {
        return esp_timer_get_time() / 1000ULL;
    }

    ErrorCode GetAnalogInputs(float **voltages)
    {
        *voltages = this->AnalogInputs;
        return ErrorCode::OK;
    }

    ErrorCode GetEncoderValue(int *value)
    {
        return this->rotenc->GetValue(value) == ESP_OK ? ErrorCode::OK : ErrorCode::GENERIC_ERROR;
    }

    ErrorCode GetFan1Rpm(float *rpm)
    {
        *rpm = std::numeric_limits<float>::quiet_NaN();
        return ErrorCode::OK;
    }

    ErrorCode SetSound(int32_t soundNumber)
    {

        if (soundNumber >= sizeof(SOUNDS) / sizeof(uint8_t *))
            soundNumber = 0;
        this->sound = soundNumber;
        mp3player.Play(SOUNDS[soundNumber], SONGS_LEN[soundNumber]);
        ESP_LOGI(TAG, "Set Sound to %d", soundNumber);

        return ErrorCode::OK;
    }

    ErrorCode GetSound(int32_t *soundNumber)
    {
        *soundNumber = this->sound;
        return ErrorCode::OK;
    }

    ErrorCode GetCO2PPM(float *co2PPM)
    {
        *co2PPM = this->airCo2PPM;
        return ErrorCode::OK;
    }

    ErrorCode GetHeaterTemperature(float *degreesCelcius)
    {
        *degreesCelcius = this->heaterTemperatureDegCel;
        return ErrorCode::OK;
    }

    ErrorCode GetAirTemperature(float *degreesCelcius)
    {
        *degreesCelcius = this->airTemperatureDegCel;
        return ErrorCode::OK;
    }

    ErrorCode GetAirPressure(float *pa)
    {
        *pa = this->airPressurePa;
        return ErrorCode::OK;
    }

    ErrorCode GetAirQuality(float *qualityPercent)
    {
        *qualityPercent = std::numeric_limits<float>::quiet_NaN();
        return ErrorCode::OK;
    }

    ErrorCode GetAirRelHumidity(float *percent)
    {
        *percent = this->airRelHumidityPercent;
        return ErrorCode::OK;
    }

    ErrorCode GetAirSpeed(float *meterPerSecond)
    {
        *meterPerSecond = this->airSpeedMeterPerSecond;
        return ErrorCode::OK;
    }

    ErrorCode GetAmbientBrightness(float *lux)
    {
        // digital value has priority
        *lux = std::isnan(this->ambientBrightnessLux_digital) ? this->ambientBrightnessLux_analog : this->ambientBrightnessLux_digital;
        return ErrorCode::OK;
    }

    ErrorCode GetWifiRssiDb(float *db)
    {
        *db = this->wifiRssiDb;
        return ErrorCode::OK;
    }

    ErrorCode SetAnalogOutput(float volts)
    {
        mp3player.OutputConstantVoltage(volts);
        return ErrorCode::OK;
    }

    ErrorCode GetFFT64(float *magnitudes64_param)
    {
        return ErrorCode::OK;
    }

    ErrorCode InitAndRun()
    {
        if (mode_io33 == MODE_IO33::FAN1_SENSE)
            return ErrorCode::NOT_YET_IMPLEMENTED;

        if (mode_multi1 != MODE_MULTI1_PIN::I2S || mode_multi23 != MODE_MULTI_2_3_PINS::I2S)
            return ErrorCode::NOT_YET_IMPLEMENTED;

        // i2s config for reading from left channel of I2S - this is standard for microphones
        /*
        i2s_config_t i2sMemsConfigLeftChannel = {};
        i2sMemsConfigLeftChannel.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
        i2sMemsConfigLeftChannel.sample_rate = SAMPLE_RATE_MICROPHONE;
        i2sMemsConfigLeftChannel.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
        i2sMemsConfigLeftChannel.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
        i2sMemsConfigLeftChannel.communication_format = I2S_COMM_FORMAT_STAND_I2S;
        i2sMemsConfigLeftChannel.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
        i2sMemsConfigLeftChannel.dma_desc_num = 8;
        i2sMemsConfigLeftChannel.dma_frame_num = 64;
        i2sMemsConfigLeftChannel.use_apll = false;
        i2sMemsConfigLeftChannel.tx_desc_auto_clear = false;
        i2sMemsConfigLeftChannel.fixed_mclk = 0;
        i2s_pin_config_t i2sPins = {};
        i2sPins.bck_io_num = PIN_I2S_SCK;
        i2sPins.ws_io_num = PIN_I2S_WS;
        i2sPins.data_out_num = I2S_PIN_NO_CHANGE;
        i2sPins.data_in_num = PIN_I2S_SD;
        i2s_driver_install(I2S_PORT_MICROPHONE, &i2sMemsConfigLeftChannel, 0, NULL);
        i2s_set_pin(I2S_PORT_MICROPHONE, &i2sPins);
        */
        gpio_reset_pin(PIN_R3_1);
        gpio_set_direction(PIN_R3_1, GPIO_MODE_INPUT);
        gpio_set_pull_mode(PIN_R3_1, GPIO_FLOATING);

        gpio_reset_pin(PIN_MOVEMENT);
        gpio_set_direction(PIN_MOVEMENT, GPIO_MODE_INPUT);
        gpio_set_pull_mode(PIN_MOVEMENT, GPIO_FLOATING);

        adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_0db, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(CHANNEL_SWITCHES, ADC_ATTEN_0db);

        gpio_reset_pin(PIN_ROTENC_A);
        gpio_set_direction(PIN_ROTENC_A, GPIO_MODE_INPUT);
        gpio_set_pull_mode(PIN_ROTENC_A, GPIO_FLOATING);

        gpio_reset_pin(PIN_ROTENC_B);
        gpio_set_direction(PIN_ROTENC_B, GPIO_MODE_INPUT);
        gpio_set_pull_mode(PIN_ROTENC_B, GPIO_PULLUP_ONLY);

        gpio_set_level(PIN_R3_ON, 0);
        gpio_reset_pin(PIN_R3_ON);
        gpio_set_direction(PIN_R3_ON, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(PIN_R3_ON, GPIO_FLOATING);

        // Servos
        if (mode_multi1 == MODE_MULTI1_PIN::SERVO1)
        {
            mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_SERVO1);
        }
        if (mode_io33 == MODE_IO33::SERVO2)
        {
            mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_SERVO2);
        }
        mcpwm_config_t pwm_config;
        pwm_config.frequency = 50; // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        pwm_config.cmpr_a = 0;     // duty cycle of PWMxA = 0
        pwm_config.cmpr_b = 0;     // duty cycle of PWMxb = 0
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config)); // Configure PWM0A & PWM0B with above settings

        // Fans
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_FAN1_DRIVE);
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PIN_FAN2_DRIVE);
        pwm_config.frequency = 50;
        pwm_config.cmpr_a = 0; // duty cycle of PWMxA = 0
        pwm_config.cmpr_b = 0; // duty cycle of PWMxb = 0
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config));

        // Heater
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PIN_HEATER);
        pwm_config.frequency = 20;
        pwm_config.cmpr_a = 0; // duty cycle of PWMxA = 0
        pwm_config.cmpr_b = 0; // duty cycle of PWMxb = 0
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config));

        // White Power LED
        ledc_timer_config_t power_ledc_timer;
        power_ledc_timer.duty_resolution = power_ledc_timer_duty_resolution; // resolution of PWM duty
        power_ledc_timer.freq_hz = 500;                                      // frequency of PWM signal
        power_ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                  // timer mode
        power_ledc_timer.timer_num = LEDC_TIMER_0;                           // timer index
        power_ledc_timer.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&power_ledc_timer));

        ledc_channel_config_t ledc_channel;
        ledc_channel.channel = LEDC_CHANNEL_0;
        ledc_channel.duty = 0;
        ledc_channel.gpio_num = PIN_LED_POWER_WHITE;
        ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
        ledc_channel.hpoint = 0;
        ledc_channel.timer_sel = LEDC_TIMER_0;
        ledc_channel.intr_type = LEDC_INTR_DISABLE;
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));



        //I2C Master
        ESP_ERROR_CHECK(I2C::Init(I2C_PORT, PIN_I2C_SCL, PIN_I2C_SDA));

        //LED Strip
        strip = new RGBLED<LED_NUMBER, DeviceType::WS2812>();
        ESP_ERROR_CHECK(strip->Init(VSPI_HOST, PIN_LED_STRIP, 2 ));
        ESP_ERROR_CHECK(strip->Clear(100));

        

        rotenc = new cRotaryEncoder(PIN_ROTENC_A, PIN_ROTENC_B, -100, 100);
        rotenc->Init();
        rotenc->Start();

        xTaskCreate(sensorTask, "sensorTask", 4096 * 4, this, 6, NULL);
        xTaskCreate(mp3Task, "mp3task", 6144 * 4, this, 16, nullptr); //Stack Size = 4096 --> Stack overflow!!
        return ErrorCode::OK;
    }

    ErrorCode BeforeLoop()
    {

        if (this->heaterTemperatureDegCel > 85)
        {
            ESP_LOGE(TAG, "Emergency Shutdown. Heater Temperature too high!!!");
            this->SetHeaterDuty(0);
            this->heaterEmergencyShutdown = true;
        }
        return ErrorCode::OK;
    }

    ErrorCode AfterLoop()
    {
        strip->Refresh(100);
        return ErrorCode::OK;
    }

    ErrorCode StartBuzzer(float freqHz)
    {
        return ErrorCode::OK;
    }

    ErrorCode EndBuzzer()
    {
        return ErrorCode::OK;
    }

    ErrorCode ColorizeLed(uint8_t ledIndex, CRGB colorCRGB)
    {
        if(ledIndex>=LED_NUMBER) return ErrorCode::INDEX_OUT_OF_BOUNDS;
        strip->SetPixel(LED_NUMBER-ledIndex-1, colorCRGB);
        return ErrorCode::OK;
    }

    ErrorCode UnColorizeAllLed()
    {
        strip->Clear(1000);
        return ErrorCode::OK;
    }

    ErrorCode SetRelayState(bool state)
    {
        gpio_set_level(PIN_R3_ON, state);
        return ErrorCode::OK;
    }

    ErrorCode SetHeaterDuty(float dutyInPercent)
    {
        if(this->heaterEmergencyShutdown) return ErrorCode::EMERGENCY_SHUTDOWN;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dutyInPercent);
        return ErrorCode::OK;
    }

    float GetHeaterState()
    {
        return mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
    }

    ErrorCode SetFan1Duty(float dutyInPercent)
    {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dutyInPercent);
        return ErrorCode::OK;
    }

    float GetFan1State()
    {
        return mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    }

    ErrorCode SetFan2Duty(float dutyInPercent)
    {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, dutyInPercent);
        return ErrorCode::OK;
    }

    float GetFan2State()
    {
        return mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
    }

    ErrorCode SetLedPowerWhiteDuty(float dutyInpercent)
    {
        if (dutyInpercent > 100)
            dutyInpercent = 100;
        uint32_t duty = (((2 ^ power_ledc_timer_duty_resolution) - 1) * dutyInpercent) / 100;
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        return ErrorCode::OK;
    }

    bool GetButtonRedIsPressed()
    {
        return GetBitIdx(this->buttonState, (uint8_t)Button::BUT_RED);
    }

    bool GetButtonEncoderIsPressed()
    {
        return GetBitIdx(this->buttonState, (uint8_t)Button::BUT_ENCODER);
    }

    bool GetButtonGreenIsPressed()
    {
        return GetBitIdx(this->buttonState, (uint8_t)Button::BUT_GREEN);
    }

    bool IsMovementDetected()
    {
        return this->movementIsDetected;
    }

    ErrorCode SetServo1Position(float angle_0_to_180)
    {
        uint32_t cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (angle_0_to_180)) / (SERVO_MAX_DEGREE)));
        esp_err_t err =  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, cal_pulsewidth);
        return err==ESP_OK?ErrorCode::OK:ErrorCode::GENERIC_ERROR;
    }

    ErrorCode SetServo2Position(float angle_0_to_180)
    {
        uint32_t cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (angle_0_to_180)) / (SERVO_MAX_DEGREE)));
        esp_err_t err =  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, cal_pulsewidth);
        return err==ESP_OK?ErrorCode::OK:ErrorCode::GENERIC_ERROR;
    }

    float GetUSBCVoltage(){
        return 20.0;
    }

    ErrorCode UpdatePinConfiguration(uint8_t* configMessage, size_t configMessagelen){
        return ErrorCode::OK;
    }

    ErrorCode GreetUserOnStartup() override{  
        for(int i=0;i<3;i++)
        {
            ColorizeLed(0, CRGB::DarkRed);
            ColorizeLed(1, CRGB::Yellow);
            ColorizeLed(2, CRGB::DarkGreen);
            ColorizeLed(3, CRGB::DarkBlue);
            AfterLoop();
            vTaskDelay(pdMS_TO_TICKS(150));
            ColorizeLed(0, CRGB::DarkBlue);
            ColorizeLed(1, CRGB::DarkGreen);
            ColorizeLed(2, CRGB::Yellow);
            ColorizeLed(3, CRGB::DarkRed);
            AfterLoop();
            vTaskDelay(pdMS_TO_TICKS(150));
        }    
        SetSound(5);
        UnColorizeAllLed();
        return ErrorCode::OK;
    }
    static void sensorTask(void *pvParameters)
    {
        HAL_labathome *hal = (HAL_labathome *)pvParameters;
        hal->SensorLoop();
    }

    static void mp3Task(void *pvParameters)
    {
        HAL_labathome *hal = (HAL_labathome *)pvParameters;
        hal->MP3Loop();
    }
};

