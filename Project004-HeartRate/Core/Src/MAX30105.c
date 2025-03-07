#include "MAX30105.h"
#include <string.h>

// ––– 저수준 I2C 인터페이스 함수들 –––
// 아래의 함수들은 사용 중인 플랫폼에 맞게 구현해야 합니다.
extern void i2cBegin(void* i2cPort);
extern void i2cSetClock(void* i2cPort, uint32_t i2cSpeed);
extern int i2cBeginTransmission(void* i2cPort, uint8_t address);
extern int i2cWrite(void* i2cPort, uint8_t data);
extern int i2cEndTransmission(void* i2cPort);
extern int i2cRequestFrom(void* i2cPort, uint8_t address, uint8_t quantity);
extern int i2cAvailable(void* i2cPort);
extern uint8_t i2cRead(void* i2cPort);

// 저수준 I2C 함수 구현 예시 (플랫폼에 따라 수정)
uint8_t readRegister8(void* i2cPort, uint8_t address, uint8_t reg) {
    i2cBeginTransmission(i2cPort, address);
    i2cWrite(i2cPort, reg);
    i2cEndTransmission(i2cPort);
    i2cRequestFrom(i2cPort, address, 1);
    if (i2cAvailable(i2cPort))
        return i2cRead(i2cPort);
    return 0;
}

void writeRegister8(void* i2cPort, uint8_t address, uint8_t reg, uint8_t value) {
    i2cBeginTransmission(i2cPort, address);
    i2cWrite(i2cPort, reg);
    i2cWrite(i2cPort, value);
    i2cEndTransmission(i2cPort);
}

// 내부 헬퍼 함수: bitMask
static void bitMask(MAX30105* sensor, uint8_t reg, uint8_t mask, uint8_t thing) {
    uint8_t originalContents = readRegister8(sensor->i2cPort, sensor->i2caddr, reg);
    originalContents &= mask;
    writeRegister8(sensor->i2cPort, sensor->i2caddr, reg, originalContents | thing);
}

// 센서 초기화 (C++의 begin()과 동일)
bool MAX30105_begin(MAX30105* sensor, void* i2cPort, uint32_t i2cSpeed, uint8_t i2caddr) {
    sensor->i2cPort = i2cPort;
    sensor->i2caddr = i2caddr;
    // I2C 초기화 (플랫폼에 맞게 구현)
    i2cBegin(i2cPort);
    i2cSetClock(i2cPort, i2cSpeed);

    // Part ID 확인
    if (MAX30105_readPartID(sensor) != MAX30105_EXPECTEDPARTID)
        return false;
    MAX30105_readRevisionID(sensor);
    return true;
}

// 인터럽트 설정 관련 함수들
uint8_t MAX30105_getINT1(MAX30105* sensor) {
    return readRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_INTSTAT1);
}
uint8_t MAX30105_getINT2(MAX30105* sensor) {
    return readRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_INTSTAT2);
}

void MAX30105_enableAFULL(MAX30105* sensor) {
    bitMask(sensor, MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_ENABLE);
}
void MAX30105_disableAFULL(MAX30105* sensor) {
    bitMask(sensor, MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_DISABLE);
}

void MAX30105_enableDATARDY(MAX30105* sensor) {
    bitMask(sensor, MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_ENABLE);
}
void MAX30105_disableDATARDY(MAX30105* sensor) {
    bitMask(sensor, MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_DISABLE);
}

void MAX30105_enableALCOVF(MAX30105* sensor) {
    bitMask(sensor, MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_ENABLE);
}
void MAX30105_disableALCOVF(MAX30105* sensor) {
    bitMask(sensor, MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_DISABLE);
}

void MAX30105_enablePROXINT(MAX30105* sensor) {
    bitMask(sensor, MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_ENABLE);
}
void MAX30105_disablePROXINT(MAX30105* sensor) {
    bitMask(sensor, MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_DISABLE);
}

void MAX30105_enableDIETEMPRDY(MAX30105* sensor) {
    bitMask(sensor, MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX30105_disableDIETEMPRDY(MAX30105* sensor) {
    bitMask(sensor, MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_DISABLE);
}

// 소프트 리셋 (softReset)
void MAX30105_softReset(MAX30105* sensor) {
    bitMask(sensor, MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);
    uint32_t startTime = millis();
    while ((millis() - startTime) < 100) {
        uint8_t response = readRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_MODECONFIG);
        if ((response & MAX30105_RESET) == 0)
            break;
        delay(1);
    }
}

void MAX30105_shutDown(MAX30105* sensor) {
    bitMask(sensor, MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_SHUTDOWN);
}

void MAX30105_wakeUp(MAX30105* sensor) {
    bitMask(sensor, MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP);
}

void MAX30105_setLEDMode(MAX30105* sensor, uint8_t mode) {
    bitMask(sensor, MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

void MAX30105_setADCRange(MAX30105* sensor, uint8_t adcRange) {
    bitMask(sensor, MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

void MAX30105_setSampleRate(MAX30105* sensor, uint8_t sampleRate) {
    bitMask(sensor, MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

void MAX30105_setPulseWidth(MAX30105* sensor, uint8_t pulseWidth) {
    bitMask(sensor, MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

void MAX30105_setPulseAmplitudeRed(MAX30105* sensor, uint8_t amplitude) {
    writeRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_LED1_PULSEAMP, amplitude);
}

void MAX30105_setPulseAmplitudeIR(MAX30105* sensor, uint8_t amplitude) {
    writeRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_LED2_PULSEAMP, amplitude);
}

void MAX30105_setPulseAmplitudeGreen(MAX30105* sensor, uint8_t amplitude) {
    writeRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_LED3_PULSEAMP, amplitude);
}

void MAX30105_setPulseAmplitudeProximity(MAX30105* sensor, uint8_t amplitude) {
    writeRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_LED_PROX_AMP, amplitude);
}

void MAX30105_setProximityThreshold(MAX30105* sensor, uint8_t threshMSB) {
    writeRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_PROXINTTHRESH, threshMSB);
}

void MAX30105_enableSlot(MAX30105* sensor, uint8_t slotNumber, uint8_t device) {
    switch (slotNumber) {
        case 1:
            bitMask(sensor, MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
            break;
        case 2:
            bitMask(sensor, MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
            break;
        case 3:
            bitMask(sensor, MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
            break;
        case 4:
            bitMask(sensor, MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
            break;
        default:
            break;
    }
}

void MAX30105_disableSlots(MAX30105* sensor) {
    writeRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_MULTILEDCONFIG1, 0);
    writeRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_MULTILEDCONFIG2, 0);
}

void MAX30105_setFIFOAverage(MAX30105* sensor, uint8_t numberOfSamples) {
    bitMask(sensor, MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numberOfSamples);
}

void MAX30105_clearFIFO(MAX30105* sensor) {
    writeRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_FIFOWRITEPTR, 0);
    writeRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_FIFOOVERFLOW, 0);
    writeRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_FIFOREADPTR, 0);
}

void MAX30105_enableFIFORollover(MAX30105* sensor) {
    bitMask(sensor, MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

void MAX30105_disableFIFORollover(MAX30105* sensor) {
    bitMask(sensor, MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_DISABLE);
}

void MAX30105_setFIFOAlmostFull(MAX30105* sensor, uint8_t numberOfSamples) {
    bitMask(sensor, MAX30105_FIFOCONFIG, MAX30105_A_FULL_MASK, numberOfSamples);
}

uint8_t MAX30105_readPartID(MAX30105* sensor) {
    return readRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_PARTID);
}

void MAX30105_readRevisionID(MAX30105* sensor) {
    sensor->revisionID = readRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_REVISIONID);
}

uint8_t MAX30105_getRevisionID(MAX30105* sensor) {
    return sensor->revisionID;
}

// 온도 읽기 함수
float MAX30105_readTemperature(MAX30105* sensor) {
    writeRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_DIETEMPCONFIG, 0x01);
    uint32_t startTime = millis();
    while ((millis() - startTime) < 100) {
        uint8_t response = readRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_INTSTAT2);
        if (response & MAX30105_INT_DIE_TEMP_RDY_ENABLE)
            break;
        delay(1);
    }
    int8_t tempInt = (int8_t)readRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_DIETEMPINT);
    uint8_t tempFrac = readRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_DIETEMPFRAC);
    return (float)tempInt + ((float)tempFrac * 0.0625);
}

float MAX30105_readTemperatureF(MAX30105* sensor) {
    float temp = MAX30105_readTemperature(sensor);
    if (temp != -999.0)
        temp = temp * 1.8 + 32.0;
    return temp;
}

// 데이터 수집 함수들
uint8_t MAX30105_available(MAX30105* sensor) {
    int numberOfSamples = sensor->sense.head - sensor->sense.tail;
    if (numberOfSamples < 0)
        numberOfSamples += STORAGE_SIZE;
    return (uint8_t)numberOfSamples;
}

uint32_t MAX30105_getRed(MAX30105* sensor) {
    if (MAX30105_safeCheck(sensor, 250))
        return sensor->sense.red[sensor->sense.head];
    else
        return 0;
}

uint32_t MAX30105_getIR(MAX30105* sensor) {
    if (MAX30105_safeCheck(sensor, 250))
        return sensor->sense.IR[sensor->sense.head];
    else
        return 0;
}

uint32_t MAX30105_getGreen(MAX30105* sensor) {
    if (MAX30105_safeCheck(sensor, 250))
        return sensor->sense.green[sensor->sense.head];
    else
        return 0;
}

uint32_t MAX30105_getFIFORed(MAX30105* sensor) {
    return sensor->sense.red[sensor->sense.tail];
}

uint32_t MAX30105_getFIFOIR(MAX30105* sensor) {
    return sensor->sense.IR[sensor->sense.tail];
}

uint32_t MAX30105_getFIFOGreen(MAX30105* sensor) {
    return sensor->sense.green[sensor->sense.tail];
}

void MAX30105_nextSample(MAX30105* sensor) {
    if (MAX30105_available(sensor)) {
        sensor->sense.tail++;
        sensor->sense.tail %= STORAGE_SIZE;
    }
}

// check() 함수 – I2C 버스트 읽기는 사용 환경에 따라 별도 구현 필요.
// 아래 코드는 개략적인 구조를 보여줍니다.
uint16_t MAX30105_check(MAX30105* sensor) {
    uint8_t readPointer = readRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_FIFOREADPTR);
    uint8_t writePointer = readRegister8(sensor->i2cPort, sensor->i2caddr, MAX30105_FIFOWRITEPTR);
    int numberOfSamples = 0;
    if (readPointer != writePointer) {
        numberOfSamples = writePointer - readPointer;
        if (numberOfSamples < 0)
            numberOfSamples += 32;
        int bytesLeftToRead = numberOfSamples * sensor->activeLEDs * 3;
        // 버스트 읽기 구현 필요:
        // 예: 버퍼를 할당하여 i2cBurstRead(sensor->i2cPort, sensor->i2caddr, MAX30105_FIFODATA, buffer, bytesLeftToRead);
        // 버퍼에서 각 LED 데이터(3바이트씩)를 파싱하여 sensor->sense 배열에 저장
        // 이 부분은 플랫폼에 따라 직접 구현하세요.
    }
    return (uint16_t)numberOfSamples;
}

bool MAX30105_safeCheck(MAX30105* sensor, uint8_t maxTimeToCheck) {
    uint32_t markTime = millis();
    while (1) {
        if ((millis() - markTime) > maxTimeToCheck)
            return false;
        if (MAX30105_check(sensor) > 0)
            return true;
        delay(1);
    }
}

// setup() 함수 – 센서 초기 기본 설정 (C++의 setup()과 동일)
void MAX30105_setup(MAX30105* sensor, uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange) {
    MAX30105_softReset(sensor);

    // FIFO 설정
    if (sampleAverage == 1)
        MAX30105_setFIFOAverage(sensor, MAX30105_SAMPLEAVG_1);
    else if (sampleAverage == 2)
        MAX30105_setFIFOAverage(sensor, MAX30105_SAMPLEAVG_2);
    else if (sampleAverage == 4)
        MAX30105_setFIFOAverage(sensor, MAX30105_SAMPLEAVG_4);
    else if (sampleAverage == 8)
        MAX30105_setFIFOAverage(sensor, MAX30105_SAMPLEAVG_8);
    else if (sampleAverage == 16)
        MAX30105_setFIFOAverage(sensor, MAX30105_SAMPLEAVG_16);
    else if (sampleAverage == 32)
        MAX30105_setFIFOAverage(sensor, MAX30105_SAMPLEAVG_32);
    else
        MAX30105_setFIFOAverage(sensor, MAX30105_SAMPLEAVG_4);
    MAX30105_enableFIFORollover(sensor);

    // 모드 설정
    if (ledMode == 3)
        MAX30105_setLEDMode(sensor, MAX30105_MODE_MULTILED);
    else if (ledMode == 2)
        MAX30105_setLEDMode(sensor, MAX30105_MODE_REDIRONLY);
    else
        MAX30105_setLEDMode(sensor, MAX30105_MODE_REDONLY);
    sensor->activeLEDs = ledMode;

    // 파티클 센싱 설정
    if (adcRange < 4096)
        MAX30105_setADCRange(sensor, MAX30105_ADCRANGE_2048);
    else if (adcRange < 8192)
        MAX30105_setADCRange(sensor, MAX30105_ADCRANGE_4096);
    else if (adcRange < 16384)
        MAX30105_setADCRange(sensor, MAX30105_ADCRANGE_8192);
    else if (adcRange == 16384)
        MAX30105_setADCRange(sensor, MAX30105_ADCRANGE_16384);
    else
        MAX30105_setADCRange(sensor, MAX30105_ADCRANGE_2048);

    if (sampleRate < 100)
        MAX30105_setSampleRate(sensor, MAX30105_SAMPLERATE_50);
    else if (sampleRate < 200)
        MAX30105_setSampleRate(sensor, MAX30105_SAMPLERATE_100);
    else if (sampleRate < 400)
        MAX30105_setSampleRate(sensor, MAX30105_SAMPLERATE_200);
    else if (sampleRate < 800)
        MAX30105_setSampleRate(sensor, MAX30105_SAMPLERATE_400);
    else if (sampleRate < 1000)
        MAX30105_setSampleRate(sensor, MAX30105_SAMPLERATE_800);
    else if (sampleRate < 1600)
        MAX30105_setSampleRate(sensor, MAX30105_SAMPLERATE_1000);
    else if (sampleRate < 3200)
        MAX30105_setSampleRate(sensor, MAX30105_SAMPLERATE_1600);
    else if (sampleRate == 3200)
        MAX30105_setSampleRate(sensor, MAX30105_SAMPLERATE_3200);
    else
        MAX30105_setSampleRate(sensor, MAX30105_SAMPLERATE_50);

    if (pulseWidth < 118)
        MAX30105_setPulseWidth(sensor, MAX30105_PULSEWIDTH_69);
    else if (pulseWidth < 215)
        MAX30105_setPulseWidth(sensor, MAX30105_PULSEWIDTH_118);
    else if (pulseWidth < 411)
        MAX30105_setPulseWidth(sensor, MAX30105_PULSEWIDTH_215);
    else if (pulseWidth == 411)
        MAX30105_setPulseWidth(sensor, MAX30105_PULSEWIDTH_411);
    else
        MAX30105_setPulseWidth(sensor, MAX30105_PULSEWIDTH_69);

    // LED 펄스 진폭 설정
    MAX30105_setPulseAmplitudeRed(sensor, powerLevel);
    MAX30105_setPulseAmplitudeIR(sensor, powerLevel);
    MAX30105_setPulseAmplitudeGreen(sensor, powerLevel);
    MAX30105_setPulseAmplitudeProximity(sensor, powerLevel);

    // Multi-LED 모드 구성
    MAX30105_enableSlot(sensor, 1, SLOT_RED_LED);
    if (ledMode > 1)
        MAX30105_enableSlot(sensor, 2, SLOT_IR_LED);
    if (ledMode > 2)
        MAX30105_enableSlot(sensor, 3, SLOT_GREEN_LED);

    MAX30105_clearFIFO(sensor);
}
