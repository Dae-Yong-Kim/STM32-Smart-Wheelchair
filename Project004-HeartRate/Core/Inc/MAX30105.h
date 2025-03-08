#ifndef MAX30105_H
#define MAX30105_H

#include <stdint.h>
#include <stdbool.h>

// 센서 FIFO 버퍼 크기 (원래 라이브러리에서는 STORAGE_SIZE로 정의됨)
#define STORAGE_SIZE 32

// I2C 기본 주소 (필요시 변경)
#define MAX30105_ADDRESS 0x57

// 레지스터 및 상수 정의 (C++ 코드의 static const들을 #define으로 변환)
#define MAX30105_INTSTAT1         0x00
#define MAX30105_INTSTAT2         0x01
#define MAX30105_INTENABLE1       0x02
#define MAX30105_INTENABLE2       0x03

#define MAX30105_FIFOWRITEPTR     0x04
#define MAX30105_FIFOOVERFLOW     0x05
#define MAX30105_FIFOREADPTR      0x06
#define MAX30105_FIFODATA         0x07

#define MAX30105_FIFOCONFIG       0x08
#define MAX30105_MODECONFIG       0x09
#define MAX30105_PARTICLECONFIG   0x0A
#define MAX30105_LED1_PULSEAMP    0x0C
#define MAX30105_LED2_PULSEAMP    0x0D
#define MAX30105_LED3_PULSEAMP    0x0E
#define MAX30105_LED_PROX_AMP     0x10
#define MAX30105_MULTILEDCONFIG1  0x11
#define MAX30105_MULTILEDCONFIG2  0x12

#define MAX30105_DIETEMPINT       0x1F
#define MAX30105_DIETEMPFRAC      0x20
#define MAX30105_DIETEMPCONFIG    0x21

#define MAX30105_PROXINTTHRESH    0x30

#define MAX30105_REVISIONID       0xFE
#define MAX30105_PARTID           0xFF    // 항상 0x15 이어야 함

// MAX30105 인터럽트 및 설정 관련 상수
#define MAX30105_INT_A_FULL_MASK          (~0x80)
#define MAX30105_INT_A_FULL_ENABLE        0x80
#define MAX30105_INT_A_FULL_DISABLE       0x00

#define MAX30105_INT_DATA_RDY_MASK        (~0x40)
#define MAX30105_INT_DATA_RDY_ENABLE      0x40
#define MAX30105_INT_DATA_RDY_DISABLE     0x00

#define MAX30105_INT_ALC_OVF_MASK         (~0x20)
#define MAX30105_INT_ALC_OVF_ENABLE       0x20
#define MAX30105_INT_ALC_OVF_DISABLE      0x00

#define MAX30105_INT_PROX_INT_MASK        (~0x10)
#define MAX30105_INT_PROX_INT_ENABLE      0x10
#define MAX30105_INT_PROX_INT_DISABLE     0x00

#define MAX30105_INT_DIE_TEMP_RDY_MASK    (~0x02)
#define MAX30105_INT_DIE_TEMP_RDY_ENABLE   0x02
#define MAX30105_INT_DIE_TEMP_RDY_DISABLE  0x00

#define MAX30105_SAMPLEAVG_MASK           (~0xE0)
#define MAX30105_SAMPLEAVG_1              0x00
#define MAX30105_SAMPLEAVG_2              0x20
#define MAX30105_SAMPLEAVG_4              0x40
#define MAX30105_SAMPLEAVG_8              0x60
#define MAX30105_SAMPLEAVG_16             0x80
#define MAX30105_SAMPLEAVG_32             0xA0

#define MAX30105_ROLLOVER_MASK            0xEF
#define MAX30105_ROLLOVER_ENABLE          0x10
#define MAX30105_ROLLOVER_DISABLE         0x00

#define MAX30105_A_FULL_MASK              0xF0

#define MAX30105_SHUTDOWN_MASK            0x7F
#define MAX30105_SHUTDOWN                 0x80
#define MAX30105_WAKEUP                   0x00

#define MAX30105_RESET_MASK               0xBF
#define MAX30105_RESET                    0x40

#define MAX30105_MODE_MASK                0xF8
#define MAX30105_MODE_REDONLY             0x02
#define MAX30105_MODE_REDIRONLY           0x03
#define MAX30105_MODE_MULTILED            0x07

#define MAX30105_ADCRANGE_MASK            0x9F
#define MAX30105_ADCRANGE_2048            0x00
#define MAX30105_ADCRANGE_4096            0x20
#define MAX30105_ADCRANGE_8192            0x40
#define MAX30105_ADCRANGE_16384           0x60

#define MAX30105_SAMPLERATE_MASK          0xE3
#define MAX30105_SAMPLERATE_50            0x00
#define MAX30105_SAMPLERATE_100           0x04
#define MAX30105_SAMPLERATE_200           0x08
#define MAX30105_SAMPLERATE_400           0x0C
#define MAX30105_SAMPLERATE_800           0x10
#define MAX30105_SAMPLERATE_1000          0x14
#define MAX30105_SAMPLERATE_1600          0x18
#define MAX30105_SAMPLERATE_3200          0x1C

#define MAX30105_PULSEWIDTH_MASK          0xFC
#define MAX30105_PULSEWIDTH_69            0x00
#define MAX30105_PULSEWIDTH_118           0x01
#define MAX30105_PULSEWIDTH_215           0x02
#define MAX30105_PULSEWIDTH_411           0x03

// Multi-LED 모드 설정 관련
#define MAX30105_SLOT1_MASK               0xF8
#define MAX30105_SLOT2_MASK               0x8F
#define MAX30105_SLOT3_MASK               0xF8
#define MAX30105_SLOT4_MASK               0x8F

#define SLOT_NONE         0x00
#define SLOT_RED_LED      0x01
#define SLOT_IR_LED       0x02
#define SLOT_GREEN_LED    0x03
#define SLOT_NONE_PILOT   0x04
#define SLOT_RED_PILOT    0x05
#define SLOT_IR_PILOT     0x06
#define SLOT_GREEN_PILOT  0x07

#define MAX30105_EXPECTEDPARTID 0x15

// MAX30105 장치 상태를 저장할 구조체
typedef struct {
    void *i2cPort;     // I2C 인터페이스 핸들 (사용 환경에 맞게 구현)
    uint8_t i2caddr;
    uint8_t activeLEDs;
    uint8_t revisionID;
    struct {
        uint8_t head;
        uint8_t tail;
        uint32_t red[STORAGE_SIZE];
        uint32_t IR[STORAGE_SIZE];
        uint32_t green[STORAGE_SIZE];
    } sense;
} MAX30105;

// 함수 프로토타입 (원래 C++ 멤버함수를 C 함수로 변환)
bool MAX30105_begin(MAX30105* sensor, void* i2cPort, uint32_t i2cSpeed, uint8_t i2caddr);
void MAX30105_setup(MAX30105* sensor, uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange);
void MAX30105_softReset(MAX30105* sensor);
void MAX30105_shutDown(MAX30105* sensor);
void MAX30105_wakeUp(MAX30105* sensor);
void MAX30105_setLEDMode(MAX30105* sensor, uint8_t mode);
void MAX30105_setADCRange(MAX30105* sensor, uint8_t adcRange);
void MAX30105_setSampleRate(MAX30105* sensor, uint8_t sampleRate);
void MAX30105_setPulseWidth(MAX30105* sensor, uint8_t pulseWidth);
void MAX30105_setPulseAmplitudeRed(MAX30105* sensor, uint8_t amplitude);
void MAX30105_setPulseAmplitudeIR(MAX30105* sensor, uint8_t amplitude);
void MAX30105_setPulseAmplitudeGreen(MAX30105* sensor, uint8_t amplitude);
void MAX30105_setPulseAmplitudeProximity(MAX30105* sensor, uint8_t amplitude);
void MAX30105_setProximityThreshold(MAX30105* sensor, uint8_t threshMSB);
uint8_t MAX30105_getINT1(MAX30105* sensor);
uint8_t MAX30105_getINT2(MAX30105* sensor);
void MAX30105_enableAFULL(MAX30105* sensor);
void MAX30105_disableAFULL(MAX30105* sensor);
void MAX30105_enableDATARDY(MAX30105* sensor);
void MAX30105_disableDATARDY(MAX30105* sensor);
void MAX30105_enableALCOVF(MAX30105* sensor);
void MAX30105_disableALCOVF(MAX30105* sensor);
void MAX30105_enablePROXINT(MAX30105* sensor);
void MAX30105_disablePROXINT(MAX30105* sensor);
void MAX30105_enableDIETEMPRDY(MAX30105* sensor);
void MAX30105_disableDIETEMPRDY(MAX30105* sensor);
float MAX30105_readTemperature(MAX30105* sensor);
float MAX30105_readTemperatureF(MAX30105* sensor);
void MAX30105_setPROXINTTHRESH(MAX30105* sensor, uint8_t val);
uint8_t MAX30105_readPartID(MAX30105* sensor);
void MAX30105_readRevisionID(MAX30105* sensor);
uint8_t MAX30105_getRevisionID(MAX30105* sensor);
uint8_t MAX30105_available(MAX30105* sensor);
uint32_t MAX30105_getRed(MAX30105* sensor);
uint32_t MAX30105_getIR(MAX30105* sensor);
uint32_t MAX30105_getGreen(MAX30105* sensor);
uint32_t MAX30105_getFIFORed(MAX30105* sensor);
uint32_t MAX30105_getFIFOIR(MAX30105* sensor);
uint32_t MAX30105_getFIFOGreen(MAX30105* sensor);
void MAX30105_nextSample(MAX30105* sensor);
uint16_t MAX30105_check(MAX30105* sensor);
bool MAX30105_safeCheck(MAX30105* sensor, uint8_t maxTimeToCheck);
void MAX30105_disableSlots(MAX30105* sensor);
void MAX30105_enableSlot(MAX30105* sensor, uint8_t slotNumber, uint8_t device);
void MAX30105_setFIFOAverage(MAX30105* sensor, uint8_t numberOfSamples);
void MAX30105_clearFIFO(MAX30105* sensor);
void MAX30105_enableFIFORollover(MAX30105* sensor);
void MAX30105_disableFIFORollover(MAX30105* sensor);
void MAX30105_setFIFOAlmostFull(MAX30105* sensor, uint8_t numberOfSamples);

// 저수준 I2C 함수 (사용 환경에 맞게 별도로 구현)
uint8_t readRegister8(void* i2cPort, uint8_t address, uint8_t reg);
void writeRegister8(void* i2cPort, uint8_t address, uint8_t reg, uint8_t value);

// millis()와 delay() 함수 또한 사용 환경에 맞게 구현해야 합니다.
extern uint32_t millis(void);
extern void delay(uint32_t ms);

#endif // MAX30105_H
