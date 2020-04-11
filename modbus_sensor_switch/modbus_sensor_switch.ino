//#include <ModbusSlave.h>
#include <Wire.h>
//#include <IWatchdog.h>

#define SLAVE_ID 1
#define RS485_BAUDRATE 115200//9600

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

//#define RS485_TX_ENABLE_PIN PA4

#define INPUT1_PIN PA0
//#define INPUT2_PIN PA4

#define OUTPUT1_PIN PA15
//#define OUTPUT2_PIN PB1

const uint16_t WATCHDOG_TIMEOUT = 10000000; //10s

const uint8_t OUT1_STATE = 0;
const uint8_t OUT2_STATE = 1;
const uint8_t OUT1_LEVEL = 0;
const uint8_t OUT2_LEVEL = 1;
const uint8_t CO2_LEVEL = 2;
const uint8_t TEMPERATURE = 3;
const uint8_t HUMIDITY = 4;
const uint8_t LIGHT_LEVEL = 5;

const uint8_t HOLDING_COUNT = 6;

bool outputState[2] = { LOW, LOW }; //{ OUT1_STATE, OUT2_STATE }
uint16_t holdingRegister[HOLDING_COUNT] = { 255, 255, 0, 0, 0, 0 }; //{ OUT1_LEVEL, OUT2_LEVEL, CO2_LEVEL, TEMPERATURE, HUMIDITY, LIGHT_LEVEL }

unsigned long INPUT_DEBOUNCE = 250UL;
bool previousInputState[2] = { HIGH, HIGH };
unsigned long lastInputTime[2] = { 0, 0 };
float temp;

//Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);

HardwareTimer *timer1;

void setup() {
 // IWatchdog.begin(WATCHDOG_TIMEOUT);

  pinMode(INPUT1_PIN, INPUT);
//  pinMode(INPUT2_PIN, INPUT);

  pinMode(OUTPUT1_PIN, OUTPUT);
//  pinMode(OUTPUT2_PIN, OUTPUT);

  initPWM();
  initI2C();
  initBH1750();
  initHTU21D();

//  slave.cbVector[CB_READ_COILS] = readDigitalOut;
//  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
//  slave.cbVector[CB_READ_HOLDING_REGISTERS] = readHolding;
//  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeHolding;

  Serial.setRx(PA3);
  Serial.setTx(PA2);
  Serial.begin(RS485_BAUDRATE);
 // slave.begin(RS485_BAUDRATE);
}

void loop() {
  readInputs();
  
  Serial.print("CO2: ");
  holdingRegister[CO2_LEVEL] = getT6703Data();
  Serial.println(holdingRegister[CO2_LEVEL]);
  Serial.print("Temp: ");

  temp = readTemperature();
  holdingRegister[TEMPERATURE] = temp * 10;
  
  Serial.println(holdingRegister[TEMPERATURE]);
  Serial.print("Humidity: ");
  holdingRegister[HUMIDITY] = readCompensatedHumidity(temp);
  Serial.println(holdingRegister[HUMIDITY]);

  Serial.print("Light: ");
  holdingRegister[LIGHT_LEVEL] = readLightLevel(); 
  Serial.println(holdingRegister[LIGHT_LEVEL]);
  
 // slave.poll();

  //delay(100);
  //IWatchdog.reload();
}

void readInputs() {
  readInput(INPUT1_PIN, OUT1_STATE);
  //readInput(INPUT2_PIN, OUT2_STATE);
}

void readInput(uint8_t pin, uint8_t output) {
  uint8_t value = digitalRead(pin);
  if ((value == LOW) && (previousInputState[output] == HIGH) && (millis() - lastInputTime[output] > INPUT_DEBOUNCE)) {
    outputState[output] = !outputState[output];
    lastInputTime[output] = millis();
    if (pin == INPUT1_PIN) {
      setOutput1();
//    } else {
//      setOutput2();  
    }
  }
  previousInputState[output] = value;
}
/*
uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  slave.writeCoilToBuffer(OUT1_STATE, outputState[OUT1_STATE]);
  slave.writeCoilToBuffer(OUT2_STATE, outputState[OUT2_STATE]);
  return STATUS_OK;
}
*/
/**
 * set digital output pins (coils).
 */
 /*
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  outputState[OUT1_STATE] = slave.readCoilFromBuffer(OUT1_STATE);
  outputState[OUT2_STATE] = slave.readCoilFromBuffer(OUT2_STATE);
  
  setOutput1();
  setOutput2();
  return STATUS_OK;
}
*/
/**
 * Handle Read Holding Registers (FC=03)
 * write back the values from eeprom (holding registers).
 */
 /*
uint8_t readHolding(uint8_t fc, uint16_t address, uint16_t length) {
  for (int i = 0; i < HOLDING_COUNT; i++) {
    slave.writeRegisterToBuffer(i, holdingRegister[i]);
  }
  return STATUS_OK;
}
*/
/**
 * Handle Write Holding Register(s) (FC=06, FC=16)
 * write data into eeprom.
 */
 /*
uint8_t writeHolding(uint8_t fc, uint16_t address, uint16_t length) {
  uint16_t value;
  for (int i = 0; i < HOLDING_COUNT; i++) {
    holdingRegister[i] = slave.readRegisterFromBuffer(i);
  }
  return STATUS_OK;
}
*/
void setOutput1() {
  uint8_t value = outputState[OUT1_STATE] ? holdingRegister[OUT1_LEVEL] : 0;  
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(OUTPUT1_PIN), PinMap_PWM));
  timer1->setPWM(channel, OUTPUT1_PIN, PWM_FREQUENCY, value);
}

void setOutput2() { 
  uint8_t value = outputState[OUT2_STATE] ? holdingRegister[OUT2_LEVEL] : 0;  
 // uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(OUTPUT2_PIN), PinMap_PWM));
 // timer1->setPWM(channel, OUTPUT2_PIN, PWM_FREQUENCY, value);
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
  if (Wire.available() != 1) return;
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
  if (Wire.endTransmission(true) != 0) return I2C_ERROR;

  delay(22);

  Wire.requestFrom(HTU21D_I2C_ADDRESS, qntRequest);
  if (Wire.available() != qntRequest) return I2C_ERROR;
  rawTemperature = Wire.read() << 8;
  rawTemperature |= Wire.read();
  checksum = Wire.read();
  
  if (checkCRC8(rawTemperature) != checksum) return I2C_ERROR;

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
    if   (data & 0x8000) data = (data << 1) ^ HTU21D_CRC8_POLYNOMINAL;
    else data <<= 1;
  }
  return data >>= 8;
}

float readHumidity() {
  uint16_t rawHumidity = 0;
  uint8_t checksum = 0;
  float humidity = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_TRIGGER_HUMD_MEASURE_HOLD);
  if (Wire.endTransmission(true) != 0) return I2C_ERROR;
  delay(4);
  Wire.requestFrom(HTU21D_I2C_ADDRESS, 3);
  if (Wire.available() != 3) return I2C_ERROR;

  rawHumidity = Wire.read() << 8;
  rawHumidity |= Wire.read();
  checksum = Wire.read();
 
  if (checkCRC8(rawHumidity) != checksum) return I2C_ERROR;

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

void initI2C() {
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
}

void initBH1750() {
  Wire.beginTransmission(BH1750_I2C_ADDRESS);
  Wire.write(BH1750_CONTINUOUS_LOW_RES_MODE);
  Wire.endTransmission();
}

float readLightLevel() {
  unsigned int level;

  Wire.beginTransmission(BH1750_I2C_ADDRESS);
  Wire.requestFrom(BH1750_I2C_ADDRESS, 2);
  level = Wire.read();
  level <<= 8;
  level |= Wire.read();
  Wire.endTransmission();

  return level / 1.2;
}
