/**
 * PIN1: D+
 * PIN3: D-
 * PIN4: Wire(I2C) SDA
 * PIN5: Wire(I2C) SCL
 * PIN6: MOVE
 * PIN7: RIGHT CLICK
 * PIN8: LEFT CLICK
 */

#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU_ADDRESS 0x68
#define ADVANCE_SCALE 1.0
#define LPF_COEF 0.9
#define LPF_ANTICOEF (1-LPF_COEF)
#define MOVE_PIN 6
#define R_CLICK_PIN 7
#define L_CLICK_PIN 8

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <Wire.h>

extern "C" {
#include "oddebug.h"
#include "usbdrv.h"
#if USE_INCLUDE
#include "usbdrv.c"
#endif

const char PROGMEM usbDescriptorHidReport[52] = {
  0x05, 0x01, // USAGE PAGE: 1 (Mouse)
  0x09, 0x02, // USAGE (PROTOCOL CODE): 2 (Mouse)
  0xa1, 0x01,
  0x09, 0x01,
  0xA1, 0x00,
  0x05, 0x09,
  0x19, 0x01,
  0x29, 0x03,
  0x15, 0x00,
  0x25, 0x01,
  0x95, 0x03,
  0x75, 0x01,
  0x81, 0x02,
  0x95, 0x01,
  0x75, 0x05,
  0x81, 0x03,
  0x05, 0x01, // USAGE PAGE: 1 (Mouse)
  0x09, 0x30, // USAGE (X)
  0x09, 0x31, // USAGE (Y)
  0x09, 0x38, // USAGE (Wheel)
  0x15, 0x81,
  0x25, 0x7F,
  0x75, 0x08, // REPORT SIZE (8 Bytes)
  0x95, 0x03,
  0x81, 0x06,
  0xC0,
  0xC0,
};

typedef struct {
  uchar buttonMask;
  char dx;
  char dy;
  char dWheel;
} report_t;

typedef struct {
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} accgy_t;

accgy_t getSensor(void);

static report_t zeroBuffer = {0, 0, 0, 0};
static report_t reportBuffer;
static uchar idleRate;

usbMsgLen_t usbFunctionSetup(uchar data[8]) {
  usbRequest_t *rq = (usbRequest_t*)data;
  if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
    switch (rq->bRequest)
    {
      case USBRQ_HID_GET_REPORT:
        usbMsgPtr = (unsigned char*) &reportBuffer;
        return sizeof(reportBuffer);
      case USBRQ_HID_GET_IDLE:
        usbMsgPtr = &idleRate;
        return 1;
      case USBRQ_HID_SET_IDLE:
        idleRate = rq->wValue.bytes[1];
        break;
      default:
        break;
    }
  }
  return 0;
}

}

static float lpv = 0.0;

inline float lowPassFilter(float value) {
  lpv = lpv * LPF_COEF + value * LPF_ANTICOEF;
  return lpv;
}

inline float highPassFilter(float value) {
  return value - lowPassFilter(value);
}

inline void applyFilter(accgy_t* data) {
  data->acc_x = highPassFilter(data->acc_x);
  data->acc_y = highPassFilter(data->acc_y);
  data->acc_z = highPassFilter(data->acc_z);
}

static float pax = 0.0;
static float pay = 0.0;
static long pt = 0;

inline void advance(void) {
  accgy_t result = getSensor();
  applyFilter(&result);
  long mt = micros();
  float ax = (pax + result.acc_x) * ((pt - mt) / 2000000);
  float ay = (pay + result.acc_y) * ((pt - mt) / 2000000);
  pax = ax;
  pay = ay;
  pt = mt;
  reportBuffer.dx += (int)(ax * ADVANCE_SCALE);
  reportBuffer.dy += (int)(ay * ADVANCE_SCALE);
}

inline void readButton(report_t* buffer) {
  if(digitalRead(L_CLICK_PIN) == HIGH) {
    buffer->buttonMask | 0x01;
  } else {
    buffer->buttonMask & 0xFE;
  }
  if(digitalRead(R_CLICK_PIN) == HIGH) {
    buffer->buttonMask | 0x02;
  } else {
    buffer->buttonMask & 0xFD;
  }
}

accgy_t getSensor(void) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 14, true);
  while(Wire.available() < 14);
  int16_t axRaw, ayRaw, azRaw, rxRaw, ryRaw, rzRaw;
  axRaw = Wire.read() << 8 | Wire.read();
  ayRaw = Wire.read() << 8 | Wire.read();
  azRaw = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();
  rxRaw = Wire.read() << 8 | Wire.read();
  ryRaw = Wire.read() << 8 | Wire.read();
  rzRaw = Wire.read() << 8 | Wire.read();
  return (accgy_t){
    axRaw / 16384.0,
    ayRaw / 16384.0,
    azRaw / 16384.0,
    rxRaw / 131.0,
    ryRaw / 131.0,
    rzRaw / 131.0};
}

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  pinMode(L_CLICK_PIN, INPUT);
  pinMode(R_CLICK_PIN, INPUT);
  pinMode(MOVE_PIN, INPUT);
}

int __attribute__((noreturn)) main(void) {
  init();
  setup();
  uchar i;
  wdt_enable(WDTO_1S);
  usbInit();
  usbDeviceDisconnect();
  i = 0;
  while(i--) {
    wdt_reset();
    _delay_ms(1);
  }
  usbDeviceConnect();
  sei();
  for(;;) {
    wdt_reset();
    usbPoll();
    if(usbInterruptIsReady()) {
      advance();

      if(digitalRead(MOVE_PIN) == LOW) {
        usbSetInterrupt((unsigned char *)&zeroBuffer, sizeof(zeroBuffer));
        readButton(&zeroBuffer);
        continue;
      }

      readButton(&reportBuffer);
      usbSetInterrupt((unsigned char *)&reportBuffer, sizeof(reportBuffer));
    }
  }
}
