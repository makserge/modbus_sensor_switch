#include <ModbusSlave.h>
#include <Wire.h>
#include <OneButton.h>

#define SLAVE_ID 10
#define RS485_BAUDRATE 9600

#define T6703_I2C_ADDRESS 0x15
#define HTU21D_I2C_ADDRESS 0x40
#define BH1750_I2C_ADDRESS 0x23

#define HTU21D_RES_RH8_TEMP12 0x01
#define HTU21D_HEATER_ON 0x04
#define HTU21D_HEATER_OFF 0xFB
#define HTU21D_USER_REGISTER_WRITE 0xE6
#define HTU21D_USER_REGISTER_READ 0xE7
#define HTU21D_TRIGGER_TEMP_MEASURE_HOLD 0xE3
#define HTU21D_CRC8_POLYNOMINAL 0x13100
#define HTU21D_TEMP_COEFFICIENT -0.15
#define HTU21D_TRIGGER_HUMD_MEASURE_HOLD 0xE5

#define BH1750_CONTINUOUS_LOW_RES_MODE 0x13

#define I2C_ERROR 0xFF

#define RS485_TX_ENABLE_PIN PA12

#define I2C_SDA PB7
#define I2C_SCL PB6

#define INPUT1_PIN PA5
#define INPUT2_PIN PA6

#define OUTPUT1_PIN PB0
#define OUTPUT2_PIN PB1

const uint8_t OUT1_STATE = 0;
const uint8_t OUT2_STATE = 1;
const uint8_t CO2_LEVEL = 2;
const uint8_t TEMPERATURE = 3;
const uint8_t HUMIDITY = 4;
const uint8_t LIGHT_LEVEL = 5;

const uint8_t HOLDING_COUNT = 6;

const uint8_t OUTPUT_STEPS = 100;
const uint8_t OUTPUT_LOG[101] =
{ 100, 100, 97, 97, 94, 94, 91, 91, 88, 88, 85, 85, 82, 82, 79, 79, 76, 76, 74, 74, 71, 71, 69, 69, 66, 66, 63, 63, 61, 61, 59, 59,
  56, 56, 54, 54, 52, 52, 50, 50, 48, 48, 46, 46, 44, 44, 42, 42, 40, 40, 38, 38, 36, 36, 34, 34, 32, 32, 30, 30, 28, 28, 26, 26, 25, 25, 24, 24, 23, 23, 22, 22, 20, 20,
  19, 19, 17, 17, 16, 16, 15, 15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5
};

const uint8_t PERIODICAL_TIMER_FREQUENCY = 1; //1HZ

uint8_t outputState[2] = { LOW, LOW }; //{ OUT1_STATE, OUT2_STATE }
uint16_t holdingRegister[HOLDING_COUNT] = { 100, 100, 0, 0, 0, 0 }; //{ OUT1_STATE, OUT2_STATE, CO2_LEVEL, TEMPERATURE, HUMIDITY, LIGHT_LEVEL }

uint8_t dimmerDirection[2] = { LOW, LOW };
uint16_t pressCounter[2] = { 0, 0 };

float temp;

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneButton button1(INPUT1_PIN, true, false);
OneButton button2(INPUT2_PIN, true, false);
HardwareTimer *timer1;

void setup() {
  initPWM();
  initButtons();
  initPeriodicalTimer();
  initI2C();
  initBH1750();
  initHTU21D();

  slave.cbVector[CB_READ_COILS] = readDigitalOut;
  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = readHolding;
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeHolding;

  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(RS485_BAUDRATE);
  slave.begin(RS485_BAUDRATE);
}

void loop() {
  button1.tick();
  button2.tick();
  slave.poll();
}

uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  for (int i = 0; i < length; i++) {
    slave.writeCoilToBuffer(i, outputState[i + address]);
  }

  return STATUS_OK;
}

/**
 * set digital output pins (coils).
 */
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  uint8_t lastOutputState1 = outputState[OUT1_STATE];
  uint8_t lastOutputState2 = outputState[OUT2_STATE];

  for (uint16_t i = 0; i < length; i++) {
    outputState[i + address] = slave.readCoilFromBuffer(i);
  }

  if (outputState[OUT1_STATE] != lastOutputState1) {
    fadeOutput(OUTPUT1_PIN, OUT1_STATE);
  }  
  if (outputState[OUT2_STATE] != lastOutputState2) {
    fadeOutput(OUTPUT2_PIN, OUT2_STATE);
  }  
  return STATUS_OK;
}
/**
 * Handle Read Holding Registers (FC=03)
 */
uint8_t readHolding(uint8_t fc, uint16_t address, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    slave.writeRegisterToBuffer(i, holdingRegister[i + address]);
  }
  return STATUS_OK;
}
/**
 * Handle Write Holding Register(s) (FC=06, FC=16)
 */
uint8_t writeHolding(uint8_t fc, uint16_t address, uint16_t length) {
  uint8_t lastOutputLevel1 = holdingRegister[OUT1_STATE];
  uint8_t lastOutputLevel2 = holdingRegister[OUT2_STATE];
  for (uint16_t i = 0; i < length; i++) {
    uint16_t value = slave.readRegisterFromBuffer(i);
    if (value <= OUTPUT_STEPS) {
      holdingRegister[i + address] = value;
    }  
  }
  if (holdingRegister[OUT1_STATE] != lastOutputLevel1) {
    outputState[OUT1_STATE] = holdingRegister[OUT1_STATE] > 0;
    setOutput(OUTPUT1_PIN, holdingRegister[OUT1_STATE]);
  }  
  if (holdingRegister[OUT2_STATE] != lastOutputLevel2) {
    outputState[OUT2_STATE] = holdingRegister[OUT2_STATE] > 0;
    setOutput(OUTPUT2_PIN, holdingRegister[OUT2_STATE]);
  }  
  return STATUS_OK;
}

uint16_t getT6703Data() {
  uint8_t rawData[5] = {0x04, 0x13, 0x8B, 0x00, 0x01};
  Wire.beginTransmission(T6703_I2C_ADDRESS);
  Wire.write(rawData, 5);
  Wire.endTransmission();
  delay(10);
  if (4 != Wire.requestFrom(T6703_I2C_ADDRESS, 4)) {
    return 0;
  }
  for (uint8_t i = 0; i < 4; i++) {
    rawData[i] = Wire.read();
  }
  if (4 != rawData[0] || 2 != rawData[1]) {
    return 0;
  }
  return ((uint16_t) rawData[2] << 8) | rawData[3];
}

void initHTU21D() {
  uint8_t userRegisterData = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_USER_REGISTER_READ);
  Wire.endTransmission(true);
  Wire.requestFrom(HTU21D_I2C_ADDRESS, 1);
  if (Wire.available() != 1) {
    return;
  }
  userRegisterData = Wire.read();

  userRegisterData &= 0x7E;
  userRegisterData |= HTU21D_RES_RH8_TEMP12;
  userRegisterData &= HTU21D_HEATER_OFF;
  
  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_USER_REGISTER_WRITE);
  Wire.write(userRegisterData);
  Wire.endTransmission(true);
}

float readTemperature() {
  int8_t qntRequest = 3;
  uint16_t rawTemperature = 0;
  uint8_t checksum = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_TRIGGER_TEMP_MEASURE_HOLD);
  if (Wire.endTransmission(true) != 0) {
    return I2C_ERROR;
  }

  delay(22);

  Wire.requestFrom(HTU21D_I2C_ADDRESS, qntRequest);
  if (Wire.available() != qntRequest) {
    return I2C_ERROR;
  }
  rawTemperature = Wire.read() << 8;
  rawTemperature |= Wire.read();
  checksum = Wire.read();
  
  if (checkCRC8(rawTemperature) != checksum) {
    return I2C_ERROR;
  }
  return (0.002681 * (float)rawTemperature - 46.85);
}

float readCompensatedHumidity(float temperature) {
  float humidity = readHumidity();
  if (humidity == I2C_ERROR || temperature == I2C_ERROR) {
    return I2C_ERROR;
  }
  if (temperature > 0 && temperature < 80) {
    humidity = humidity + (25.0 - temperature) * HTU21D_TEMP_COEFFICIENT;
  }
  return humidity;
}

uint8_t checkCRC8(uint16_t data) {
  for (uint8_t bit = 0; bit < 16; bit++) {
    if (data & 0x8000) {
      data = (data << 1) ^ HTU21D_CRC8_POLYNOMINAL;
    } else {
      data <<= 1;
    }
  }
  return data >>= 8;
}

float readHumidity() {
  uint16_t rawHumidity = 0;
  uint8_t checksum = 0;
  float humidity = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_TRIGGER_HUMD_MEASURE_HOLD);
  if (Wire.endTransmission(true) != 0) {
    return I2C_ERROR;
  }
  delay(4);
  Wire.requestFrom(HTU21D_I2C_ADDRESS, 3);
  if (Wire.available() != 3) {
    return I2C_ERROR;
  }

  rawHumidity = Wire.read() << 8;
  rawHumidity |= Wire.read();
  checksum = Wire.read();
 
  if (checkCRC8(rawHumidity) != checksum) {
    return I2C_ERROR;
  }

  rawHumidity ^= 0x02;
  humidity = (0.001907 * (float)rawHumidity - 6);
  
  if (humidity < 0) {
    humidity = 0;
  } else if (humidity > 100) {
    humidity = 100;
  }
  return humidity;
}

void initPWM() {
  TIM_TypeDef *instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(OUTPUT1_PIN), PinMap_PWM);
  timer1 = new HardwareTimer(instance);
}

void initPeriodicalTimer() {
  HardwareTimer *timer = new HardwareTimer(TIM2);
  timer->setOverflow(PERIODICAL_TIMER_FREQUENCY, HERTZ_FORMAT);
  timer->attachInterrupt(updateSensors);
  timer->resume();
}

void initI2C() {
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
}

void initBH1750() {
  Wire.beginTransmission(BH1750_I2C_ADDRESS);
  Wire.write(BH1750_CONTINUOUS_LOW_RES_MODE);
  Wire.endTransmission();
}

void updateSensors() {
  holdingRegister[CO2_LEVEL] = getT6703Data();
  
  temp = readTemperature();
  holdingRegister[TEMPERATURE] = temp * 10;
  
  holdingRegister[HUMIDITY] = readCompensatedHumidity(temp);  
  holdingRegister[LIGHT_LEVEL] = readLightLevel(); 
}

float readLightLevel() {
  Wire.beginTransmission(BH1750_I2C_ADDRESS);
  Wire.requestFrom(BH1750_I2C_ADDRESS, 2);
  uint16_t level = Wire.read();
  level <<= 8;
  level |= Wire.read();
  Wire.endTransmission();

  return level / 1.2;
}

void initButtons() {
  pinMode(INPUT1_PIN, INPUT);
  pinMode(INPUT2_PIN, INPUT);

  pinMode(OUTPUT1_PIN, OUTPUT);
  pinMode(OUTPUT2_PIN, OUTPUT);

  setOutput(OUTPUT1_PIN, 0);
  setOutput(OUTPUT2_PIN, 0);

  button1.attachClick(clickButton1);
  button1.attachDuringLongPress(duringLongPressButton1);
  button2.attachClick(clickButton2);
  button2.attachDuringLongPress(duringLongPressButton2);
}

void setOutput(uint8_t pin, uint8_t value) {
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));
  timer1->setPWM(channel, pin, PWM_FREQUENCY, OUTPUT_LOG[value]);
}

void clickButton1() {
  outputState[OUT1_STATE] = !outputState[OUT1_STATE];
  fadeOutput(OUTPUT1_PIN, OUT1_STATE);
}

void clickButton2() {
  outputState[OUT2_STATE] = !outputState[OUT2_STATE];
  fadeOutput(OUTPUT2_PIN, OUT2_STATE);
}

void duringLongPressButton1() {
  duringLongPressButton(OUTPUT1_PIN, OUT1_STATE);
}

void duringLongPressButton2() {
  duringLongPressButton(OUTPUT2_PIN, OUT2_STATE);
}

void fadeOutput(uint8_t pin, uint8_t button) {
  uint8_t value = holdingRegister[button];
  if (outputState[button]) {
    for (uint8_t i = 0; i <= value; i++) {
      setOutput(pin, i);
      delay(15);
    }
  } else {
    for (int8_t i = value; i >= 0; i--) {
      setOutput(pin, i);
      delay(15);
    }
  }
}

void duringLongPressButton(uint8_t pin, uint8_t button) {
  pressCounter[button]++;
  if (pressCounter[button] == 10000) {
    pressCounter[button] = 0;

    if (dimmerDirection[button]) {
      holdingRegister[button] += 1;
    } else {
      holdingRegister[button] -= 1;
    }
    if (holdingRegister[button] <= 0) {
      holdingRegister[button] = 0;
      dimmerDirection[button] = HIGH;
    }
    else if (holdingRegister[button] >= OUTPUT_STEPS) {
      holdingRegister[button] = OUTPUT_STEPS;
      dimmerDirection[button] = LOW;
    }
    setOutput(pin, holdingRegister[button]);
  }
  outputState[button] = HIGH;
}
