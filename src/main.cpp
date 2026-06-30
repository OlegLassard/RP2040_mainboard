#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// --- НАСТРОЙКИ ПИНОВ СВЕТОДИОДОВ (ШИМ) ---
const uint8_t PIN_LED1 = 13;
const uint8_t PIN_LED2 = 15;

// --- НАСТРОЙКИ ПИНОВ ЛАЗЕРОВ И OCULUS ---
const uint8_t PIN_LASER1 = 26;
const uint8_t PIN_LASER2 = 27;
const uint8_t PIN_LASER3 = 28;
const uint8_t PIN_LASER4 = 29;

const uint8_t PIN_OCULUS = 14; 

// --- НАСТРОЙКИ ПИНОВ МОТОРОВ (A4950) ---
const uint8_t M1_IN1 = 6;
const uint8_t M1_IN2 = 7;
const uint8_t M1_VREF = 8; 

const uint8_t M2_IN1 = 10;
const uint8_t M2_IN2 = 11;
const uint8_t M2_VREF = 12; 

// --- НАСТРОЙКИ ЯРКОСТИ, ТОКА И ПРОТОКОЛА UART ---
#define BRIGHTNESS_STEP 0.02f 
#define MOTOR_CURRENT_A 1.3f  

#define OPEN 1
#define CLOSE -1
#define CW 1
#define CCW -1

#define ROT 1
#define KLESH 2

const uint8_t PACKET_SIZE = 32; 
uint8_t rxBuffer[PACKET_SIZE];  
uint8_t rxIndex = 0;            

// Целевые значения для ШИМ светодиодов (0-255)
uint8_t target_LED1 = 0; 
uint8_t target_LED2 = 0; 

float globalBrightness = 0.0; 
float BrightnessRGB = 0.0; 
float BrightnessSide = 0.0; 
bool needUpdate = true;       

// Переменные для обработки кнопок-триггеров
bool isLaserOn = false;
uint8_t lastLaserBit = 0;

bool isOculusOn = false;
uint8_t lastOculusBit = 0;

// --- ДАННЫЕ ДАТЧИКА ---
unsigned long previousMpuMillis = 0;
const uint32_t MPU_INTERVAL_MS = 50; 
int16_t currentAngleX = 0; // Хранение текущего угла по оси X

// Объекты
Adafruit_MPU6050 mpu;

// --- ФУНКЦИИ УПРАВЛЕНИЯ ЖЕЛЕЗОМ ---

void updateLEDs() {
    uint8_t out_1 = (uint8_t)(target_LED1 * globalBrightness);
    uint8_t out_2 = (uint8_t)(target_LED2 * globalBrightness);

    analogWrite(PIN_LED1, out_1);
    analogWrite(PIN_LED2, out_2);
}

void setLasers(bool state) {
    digitalWrite(PIN_LASER1, state ? HIGH : LOW);
    digitalWrite(PIN_LASER2, state ? HIGH : LOW);
    digitalWrite(PIN_LASER3, state ? HIGH : LOW);
    digitalWrite(PIN_LASER4, state ? HIGH : LOW);
}

void setOculus(bool state) {
    if (state) {
        pinMode(PIN_OCULUS, OUTPUT);
        digitalWrite(PIN_OCULUS, LOW);
    } else {
        pinMode(PIN_OCULUS, INPUT); // High-Z
    }
}

void setMotor(uint8_t motor, int dir, float current_A) {
    if (current_A > 2.2f) current_A = 2.2f;
    if (current_A < 0.0f) current_A = 0.0f;

    uint8_t pwmDuty = (uint8_t)(current_A * 115.909f); 

    uint8_t in1_pin = (motor == 1) ? M1_IN1 : M2_IN1;
    uint8_t in2_pin = (motor == 1) ? M1_IN2 : M2_IN2;
    uint8_t vref_pin = (motor == 1) ? M1_VREF : M2_VREF;

    analogWrite(vref_pin, pwmDuty);

    if (dir == 1) {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
    } else if (dir == -1) {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
    } else {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, HIGH);
    }
}

// --- ФУНКЦИЯ ДАТЧИКА ---
void handleMPU6050() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMpuMillis >= MPU_INTERVAL_MS) {
        previousMpuMillis = currentMillis;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Вычисляем угол наклона (крен/roll) по оси X. Диапазон: -180 до +180 градусов.
        // Используем Y и Z векторы ускорения свободного падения
        float angleF = atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
        currentAngleX = (int16_t)angleF;
    }
}

// --- ФУНКЦИЯ ОТПРАВКИ ОТВЕТА МАСТЕРУ ---
void sendResponse() {
    // Создаем буфер 64 байта и сразу заполняем его нулями
    uint8_t txBuffer[64] = {0}; 

    // Байт 41: Флаги состояния (Бит 0: Лазеры, Бит 1: OCULUS)
    if (isLaserOn)  txBuffer[41] |= 0x01;
    if (isOculusOn) txBuffer[41] |= 0x02;

    // Байт 43: Интенсивность света (0-100%)
    txBuffer[43] = (uint8_t)(globalBrightness * 100);

    //Затычки для управления яркостью RGB и боковых светодиодов
    txBuffer[42] = (uint8_t)(BrightnessRGB * 100);
    txBuffer[44] = (uint8_t)(BrightnessSide * 100);

    // Байты 54 и 55: Угол наклона по оси X (знаковое 16-битное число)
    // Разбиваем на два байта: 54 - старший (High), 55 - младший (Low)
    txBuffer[54] = (currentAngleX >> 8) & 0xFF;
    txBuffer[55] = currentAngleX & 0xFF;

    // Отправляем пакет в UART
    Serial1.write(txBuffer, 64);
}

// --- ПАРСЕР UART ---
void handleUART() {
    while (Serial1.available() > 0) {
        uint8_t incomingByte = Serial1.read();

        // Проверка жесткого заголовка
        if (rxIndex == 0 && incomingByte != 0x96) { continue; }
        if (rxIndex == 1 && incomingByte != 0xAA) { rxIndex = 0; continue; }
        if (rxIndex == 2 && incomingByte != 0xBE) { rxIndex = 0; continue; }

        rxBuffer[rxIndex++] = incomingByte;

        // Если собрали весь пакет (32 байта)
        if (rxIndex >= PACKET_SIZE) {
            
            // --- 1. Обновляем ШИМ ---
            if (target_LED1 != rxBuffer[20] || target_LED2 != rxBuffer[21]) {
                target_LED1 = rxBuffer[20];
                target_LED2 = rxBuffer[21];
                needUpdate = true;
            }

            // --- 2. Управление OCULUS (байт 3, БИТ 0 - маска 0x01) ---
            uint8_t currentOculusBit = rxBuffer[3] & 0x01;
            if (currentOculusBit && !lastOculusBit) { 
                isOculusOn = !isOculusOn; 
                setOculus(isOculusOn);
            }
            lastOculusBit = currentOculusBit;

            // --- 3. Управление лазерами (байт 3, БИТ 1 - маска 0x02) ---
            uint8_t currentLaserBit = rxBuffer[3] & 0x02;
            if (currentLaserBit && !lastLaserBit) { 
                isLaserOn = !isLaserOn; 
                setLasers(isLaserOn);
            }
            lastLaserBit = currentLaserBit; 

            // --- 4. Логика изменения яркости ---
            if (rxBuffer[16] == 0x02) {
                uint8_t reg3 = rxBuffer[3];
                float oldBrightness = globalBrightness;
                
                if ((reg3 & 0x10) && (reg3 & 0x40)) {
                    if (globalBrightness > 0.0f) globalBrightness = 0.0f;
                    else globalBrightness = 1.0f;
                } else {
                    if (reg3 & 0x10) globalBrightness += BRIGHTNESS_STEP;
                    if (reg3 & 0x40) globalBrightness -= BRIGHTNESS_STEP;
                }

                if (globalBrightness > 1.0f) globalBrightness = 1.0f;
                if (globalBrightness < 0.0f) globalBrightness = 0.0f;

                if (oldBrightness != globalBrightness) {
                    needUpdate = true;
                }
            }

            //Затычка для управления яркостью RGB
            if (rxBuffer[16] == 0x01) {
                uint8_t reg3 = rxBuffer[3];
                
                if ((reg3 & 0x10) && (reg3 & 0x40)) {
                    if (BrightnessRGB > 0.0f) BrightnessRGB = 0.0f;
                    else BrightnessRGB = 1.0f;
                } else {
                    if (reg3 & 0x10) BrightnessRGB += BRIGHTNESS_STEP;
                    if (reg3 & 0x40) BrightnessRGB -= BRIGHTNESS_STEP;
                }

                if (BrightnessRGB > 1.0f) BrightnessRGB = 1.0f;
                if (BrightnessRGB < 0.0f) BrightnessRGB = 0.0f;
            }

            //Затычка для управления яркостью боковых светодиодов
            if (rxBuffer[16] == 0x05) {
                uint8_t reg3 = rxBuffer[3];
                
                if ((reg3 & 0x10) && (reg3 & 0x40)) {
                    if (BrightnessSide > 0.0f) BrightnessSide = 0.0f;
                    else BrightnessSide = 1.0f;
                } else {
                    if (reg3 & 0x10) BrightnessSide += BRIGHTNESS_STEP;
                    if (reg3 & 0x40) BrightnessSide -= BRIGHTNESS_STEP;
                }

                if (BrightnessSide > 1.0f) BrightnessSide = 1.0f;
                if (BrightnessSide < 0.0f) BrightnessSide = 0.0f;
            }


            // --- 5. Управление моторами ---
            int dirM1 = 0; 
            if (rxBuffer[5] & 0x01) {
                dirM1 = 1;       // Открыть
            } else if (rxBuffer[5] & 0x04) {
                dirM1 = -1;      // Закрыть
            }
            setMotor(1, dirM1, MOTOR_CURRENT_A);

            int dirM2 = 0; 
            if (rxBuffer[5] & 0x02) {
                dirM2 = 1;       // Влево
            } else if (rxBuffer[5] & 0x08) {
                dirM2 = -1;      // Вправо
            }
            setMotor(2, dirM2, MOTOR_CURRENT_A);
            
            // --- 6. Отправка ответа мастеру ---
            sendResponse();

            // Сбрасываем индекс для приема следующей посылки
            rxIndex = 0;
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Настройка ШИМ
    analogWriteFreq(20000); 
    analogWriteResolution(8);

    // Светодиоды
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);

    // Лазеры и OCULUS
    pinMode(PIN_LASER1, OUTPUT);
    pinMode(PIN_LASER2, OUTPUT);
    pinMode(PIN_LASER3, OUTPUT);
    pinMode(PIN_LASER4, OUTPUT);
    setLasers(false); 
    setOculus(false); 

    // Моторы
    pinMode(M1_IN1, OUTPUT);
    pinMode(M1_IN2, OUTPUT);
    pinMode(M1_VREF, OUTPUT);

    pinMode(M2_IN1, OUTPUT);
    pinMode(M2_IN2, OUTPUT);
    pinMode(M2_VREF, OUTPUT);

    setMotor(1, 0, 0.0f);
    setMotor(2, 0, 0.0f);

    // Датчик MPU-6050
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();

    if (!mpu.begin()) {
        Serial.println("Не удалось найти чип MPU6050!");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    // UART
    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.begin(38400);

    updateLEDs();
}

void loop() {
    handleUART();
    handleMPU6050();
        
    if (needUpdate) {
        updateLEDs();
        needUpdate = false;
    }
}