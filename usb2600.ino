/*
 * Needs Roger's libmaple core: https://github.com/rogerclarkmelbourne/Arduino_STM32
 * And version 0.99 or later of https://github.com/arpruss/USBComposite_stm32f1/
 * (The version bundled in Roger's core may be too old.)
 */

#include <USBComposite.h>
#include "debounce.h"

const uint32 paddleMaxResistance = 1000000;
const uint32 paddleGroundResistor = 100000; // paddleGroundResistor should evenly divide paddleMaxResistance

// designed for an stm32f103c8t6 blue pill

// Atari pin 7 -- VCC
// Atari pin 3 -- PA2 // left paddle button, joystick left
// Atari pin 4 -- PA4 // right paddle button, joystick right
// Atari pin 9 -- PA1 // pot 1
// Atari pin 5 -- PA3 // pot 2
// Atari pin 8 -- GND
// Atari pin 6 -- PA5 // fire
// Atari pin 1 -- PA6 // joystick up
// Atari pin 2 -- PA7 // joystick down
// 100K resistors between Atari pin 9 and GND and between Atari pin 5 and GND

#define PRODUCT_ID 0x4BA2
#define LED PC13
#define ANALOG1 PA1
#define ANALOG2 PA3
#define JOY_LEFT  PA2 // also left paddle button
#define JOY_RIGHT PA4 // also right paddle paddle
#define JOY_UP    PA6 
#define JOY_DOWN  PA7
#define JOY_FIRE  PA5

#undef SUPPORT_X360 // not fully supported
#undef SERIAL_DEBUG

// at READ_ITERATIONS=1, 12 ms
// 20: 12 ms
// 40: 12 ms
// 80: 12 ms
// 100: 16 ms
// 800: 36 ms

#define NUM_EXTRA 0  // number of extra keys that emulate keyboard presses
#if NUM_EXTRA
const uint16 extraKeys[NUM_EXTRA] = { KEY_F1, KEY_F2, '[', ']' };
#endif
#define NUM_PADDLES 2
#define HYSTERESIS 20 // shifts smaller than this are rejected
#define MAX_HYSTERESIS_REJECTIONS 8 // unless we've reached this many of them, and then we use an average
#define READ_ITERATIONS 80

#define NO_VALUE 0xDEADBEEFul

// modified from Stelladaptor
uint8 dualAxis_desc[] = {
  0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
  0x15, 0x00,                    // LOGICAL_MINIMUM (0)
  0x09, 0x04,                    // USAGE (Joystick)
  0xa1, 0x01,                    // COLLECTION (Application)
  0x85, 1,                       /*    REPORT_ID */ // not present in official Stelladaptor
  0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
  0x26, 0xff, 0x03,              //   LOGICAL_MAXIMUM (1023) // 255 for official Stelladaptor
  0x75, 0x0A,                    //   REPORT_SIZE (10) // 8 for official Stelladaptor
  //0x95, 0x01,                    //   REPORT_COUNT (1) /* byte 0 unused */ // official Stelladaptor
  //0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)  // official Stelladaptor
  0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
  0x09, 0x01,                    //   USAGE (Pointer)
  0xa1, 0x00,                    //   COLLECTION (Physical)
  0x09, 0x30,                    //     USAGE (X)
  0x09, 0x31,                    //     USAGE (Y)
  0x95, 0x02,                    //     REPORT_COUNT (2)
  0x81, 0x02,                    //     INPUT (Data,Var,Abs)
  0xc0,                          //     END_COLLECTION
  0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
  0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
  0x05, 0x09,                    //   USAGE_PAGE (Button)
  0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
  0x29, 0x02,                    //   USAGE_MAXIMUM (Button 2)
  0x55, 0x00,                    //   UNIT_EXPONENT (0)
  0x65, 0x00,                    //   UNIT (None)
  0x75, 0x01,                    //   REPORT_SIZE (1)
  0x95, 0x02,                    //   REPORT_COUNT (2)
  0x81, 0x02,                    //   INPUT (Data,Var,Abs)
  0x95, 0x02,                    //   REPORT_COUNT (2) // 6 for official Stelladaptor
  0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
  0xc0                           // END_COLLECTION
};

HIDReportDescriptor dualAxis = {
  dualAxis_desc,
  sizeof(dualAxis_desc)
};

typedef struct {
  uint8_t reportID;
  unsigned x: 10;
  unsigned y: 10;
  uint8_t button1: 1;
  uint8_t button2: 1;
  uint8_t padding: 2;
} __packed SimpleJoystickReport_t;

class HIDSimpleJoystick : public HIDReporter {
  public:
    SimpleJoystickReport_t joyReport;
    HIDSimpleJoystick(USBHID& HID, uint8_t reportID = HID_JOYSTICK_REPORT_ID)
      : HIDReporter(HID, &dualAxis, (uint8_t*) & joyReport, sizeof(joyReport), reportID) {
      joyReport.button1 = 0;
      joyReport.button2 = 0;
      joyReport.x = 512;
      joyReport.y = 512;
    }
};

uint16 analogRead2(uint8 pin) {
  return adc_read(pin == ANALOG1 ? &adc1 : &adc2, PIN_MAP[pin].adc_channel);
}

#ifdef SERIAL_DEBUG
USBCompositeSerial debug;
#define DEBUG(...) debug.println(__VA_ARGS__);
#else
#define DEBUG(...)
#endif

class AnalogPort {
  public:
    uint32 port;
    uint32 oldValue = NO_VALUE;
    uint32 rejectedCount = 0;
    uint32 rejectedSum = 0;
    uint32 getValue() {
      uint32 v = 0;

//      nvic_globalirq_disable();
      for (uint32 i = 0 ; i < READ_ITERATIONS ; i++)
        v += analogRead2(port);
//      nvic_globalirq_enable();
      v = (v + READ_ITERATIONS / 2) / READ_ITERATIONS;

      if (oldValue != NO_VALUE && v != oldValue && v < oldValue + HYSTERESIS && oldValue < v + HYSTERESIS) {
        if (rejectedCount > 0) {
          rejectedCount++;
          rejectedSum += v;
          if (rejectedCount >= MAX_HYSTERESIS_REJECTIONS) {
            v = (rejectedSum + MAX_HYSTERESIS_REJECTIONS / 2) / MAX_HYSTERESIS_REJECTIONS;
            rejectedCount = 0;
          }
          else {
            v = oldValue;
          }
        }
        else {
          rejectedCount = 1;
          rejectedSum = v;
          v = oldValue;
        }
      }
      else {
        rejectedCount = 0;
      }

      oldValue = v;
      return 4095 - v;
    };

    AnalogPort(uint32 _port) {
      port = _port;
    };
};

AnalogPort analog1(ANALOG1);
AnalogPort analog2(ANALOG2);
AnalogPort* analog[2] = { &analog1, &analog2 };
Debounce digital1(PA2);
Debounce digital2(PA4);
#define NUM_DIGITAL 5
Debounce joyLeft(JOY_LEFT);
Debounce joyRight(JOY_RIGHT);
Debounce joyUp(JOY_UP);
Debounce joyDown(JOY_DOWN);
Debounce joyFire(JOY_FIRE); 
Debounce* digital[NUM_DIGITAL] = { &joyLeft, &joyRight, &joyUp, &joyDown, &joyFire };
bool havePaddle = false;

USBHID HID;
HIDSimpleJoystick joy(HID);

void setup() {
  // default is 55_5
  adc_set_sample_rate(ADC1, ADC_SMPR_239_5);
  adc_set_sample_rate(ADC2, ADC_SMPR_239_5);

  for (uint32 i = 0 ; i < 2; i++) 
    pinMode(analog[i]->port, INPUT_ANALOG);

  for (uint32 i = 0 ; i < NUM_DIGITAL ; i++) 
    pinMode(digital[i]->pin, INPUT_PULLDOWN);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);

  HID.clear();
  USBComposite.setVendorId(0x04d8);
  USBComposite.setProductId(0xbeef);
  USBComposite.setManufacturerString("Grand Idea Studio");
  USBComposite.setProductString("Stelladaptor 2600-to-USB Interface");
  HID.begin();

  while (!USBComposite);
}

uint32 valueToPot(uint32 value) {
  // value = (4095 * potResistance + 0 * paddleGroundResistor) / (potResistance + paddleGroundResistor)
  // value * (potResistance + paddleGroundResistor) = 4095 * potResistance
  // (4095 - value) * potResistance = value * paddleGroundResistor
  // potResistance = value * paddleGroundResistor / (4095 - value)
  // axis = potResistance / paddleMaxResistance * 1023
  // axis = 1023 * value * paddleGroundResistor / (4095 - value) / paddleMaxResistance

  if (value == 4095)
    return 1023;
  return 1023 * value / ((4095 - value) * paddleMaxResistance / paddleGroundResistor);
}

void loop() {
  uint32 pots[2];

  havePaddle = false;
  for (uint32 i = 0 ; i < 2; i++ ) {
    uint32 value = analog[i]->getValue();
    if (value > 4095/(paddleMaxResistance/paddleGroundResistance)/2) {
      havePaddle = true;
      digitalWrite(LED, 0);
    }
    pots[i] = valueToPot(value);
  }

  if (havePaddle) {
    if (joyLeft.getState())
      joy.joyReport.button1 = 1;
    else
      joy.joyReport.button1 = 0;
    if (joyRight.getState())
      joy.joyReport.button2 = 1;
    else
      joy.joyReport.button2 = 0;
    joy.joyReport.x = pots[0];
    joy.joyReport.y = pots[1];
  }
  else {
    bool left = joyLeft.getState();
    bool right = joyRight.getState();
    bool up = joyUp.getState();
    bool down = joyDown.getState();
    bool fire = joyFire.getState();

    if (left) 
      joy.joyReport.x = 0;
    else if (right)
      joy.joyReport.x = 1023;
    else
      joy.joyReport.x = 512;

    if (up)
      joy.joyReport.y = 0;
    else if (down)
      joy.joyReport.y = 1023;
    else
      joy.joyReport.y = 512;

    if (fire) 
      joy.joyReport.button1 = 1;
    else
      joy.joyReport.button1 = 0;
  }
  
  joy.sendReport();
}

