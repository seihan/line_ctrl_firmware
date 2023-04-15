#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <VescUart.h>

#define SERVICE_UUID "0058545f-5f5f-5f52-4148-435245574f50"
#define MOTORRIGHTCHAR_UUID "0058545f-5f5f-5f52-4148-435245574f51"  // MotorRight
#define MOTORLEFTCHAR_UUID "0058545f-5f5f-5f52-4148-435245574f52"   // MotorLeft
#define STEERINGCHAR_UUID "0058545f-5f5f-5f52-4148-435245574f53"
#define POWERCHAR_RX_UUID "0058545f-5f5f-5f52-4148-435245574f54"  // MotorMain
#define POWERCHAR_TX_UUID "0058545f-5f5f-5f52-4148-435245574f55"  // POWERCHAR____TX


/*
  IBT-2 Motor Control Board driven by Arduino.

  Speed and direction controlled by a potentiometer attached to analog input 0.
  One side pin of the potentiometer (either one) to ground; the other side pin to +5V

  Connection to the IBT-2 board:
  IBT-2 pin 1 (RPWM) to esp32-ttgo pin 27(PWM)
  IBT-2 pin 2 (LPWM) to esp32-ttgo pin 25(PWM)
  IBT-2 pins 4 (R_EN), 3 (L_EN), 5V (VCC) to esp32-ttgo 5V pin
  IBT-2 pin GND (GND) to esp32-ttgo GND
  IBT-2 pins 5 (R_IS) and 6 (L_IS) not connected
*/

// Vesc
VescUart vesc;

uint32_t value = 0;

// PWM

enum Pwm {
  FREQ = 10000,
  RESOLUTION = 8,
  M1_FWD_CH = 0,
  M1_BWD_CH = 1,
  M2_FWD_CH = 2,
  M2_BWD_CH = 3,
};

enum Pins {
  M1_RPWM = 33,
  M1_LPWM = 26,
  M2_RPWM = 23,
  M2_LPWM = 18,
  M1_EN = 19,
  M2_EN = 5,
};


// Just keep current movement settings in global variables
struct state {
  bool forward;
  int16_t speed;
} motors[2];

void set_speed_forward_m1(int16_t new_speed) {
  motors[0].forward = true;
  motors[0].speed = new_speed;
  ledcWrite(M1_BWD_CH, 0);
  ledcWrite(M1_FWD_CH, new_speed);
}

void set_speed_backward_m1(int16_t new_speed) {
  motors[0].forward = false;
  motors[0].speed = new_speed;
  ledcWrite(M1_FWD_CH, 0);
  ledcWrite(M1_BWD_CH, new_speed);
}

void set_speed_forward_m2(int16_t new_speed) {
  motors[1].forward = true;
  motors[1].speed = new_speed;
  ledcWrite(M2_BWD_CH, 0);
  ledcWrite(M2_FWD_CH, new_speed);
}

void set_speed_backward_m2(int16_t new_speed) {
  motors[1].forward = false;
  motors[1].speed = new_speed;
  ledcWrite(M2_FWD_CH, 0);
  ledcWrite(M2_BWD_CH, new_speed);
}


// Full STOP
void full_stop() {
  full_stop_m1();
  full_stop_m2();
}

void full_stop_m1() {
  ledcWrite(M1_FWD_CH, 0);
  ledcWrite(M1_BWD_CH, 0);
}

void full_stop_m2() {
  ledcWrite(M2_FWD_CH, 0);
  ledcWrite(M2_BWD_CH, 0);
}
// BLE

bool deviceConnected = false;
BLECharacteristic *pPowerCharTx = NULL;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
  }
};

class MotorRightCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    if (value.length() > 0) {
      int intValue = value.toInt();
      if (intValue < 0) {
        set_speed_backward_m1(-intValue);
      } else {
        set_speed_forward_m1(intValue);
      }
    }
  }
};

class MotorLeftCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    if (value.length() > 0) {
      int intValue = value.toInt();
      if (intValue < 0) {
        set_speed_backward_m2(-intValue);
      } else {
        set_speed_forward_m2(intValue);
      }
    }
  }
};

class SteeringCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    if (value.length() > 0) {
      int intValue = value.toInt();
      if (intValue < 0) {
        set_speed_backward_m1(-intValue);
        set_speed_forward_m2(-intValue);
      } else {
        set_speed_forward_m1(intValue);
        set_speed_backward_m2(intValue);
      }
    }
  }
};

class PowerCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    if (value.length() > 0) {
      int intValue = value.toInt();
      intValue = map(intValue, -255, 255, 0, 10);
      if (intValue < 10 && intValue > 0) {
          vesc.setCurrent(intValue);
      }
    }
  }
};

// VESC
void send_vesc_values() {
  int len = sizeof(vesc.data);
  if (deviceConnected) {
    pPowerCharTx->setValue((uint8_t *)&vesc.data, len);
    pPowerCharTx->notify();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  vesc.setSerialPort(&Serial);
  // Set motor driver enable pins high.
  pinMode(M1_EN, OUTPUT);
  digitalWrite(M1_EN, HIGH);
  pinMode(M2_EN, OUTPUT);
  digitalWrite(M2_EN, HIGH);
  // Configure motor driver PWM pins.
  ledcSetup(M1_FWD_CH, FREQ, RESOLUTION);
  ledcSetup(M2_FWD_CH, FREQ, RESOLUTION);
  ledcSetup(M1_BWD_CH, FREQ, RESOLUTION);
  ledcSetup(M2_BWD_CH, FREQ, RESOLUTION);
  ledcAttachPin(M1_RPWM, M1_FWD_CH);
  ledcAttachPin(M1_LPWM, M1_BWD_CH);
  ledcAttachPin(M2_RPWM, M2_FWD_CH);
  ledcAttachPin(M2_LPWM, M2_BWD_CH);

  full_stop();

  // bluetooth
  BLEDevice::init("LineCtrl");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pMotorRightChar = pService->createCharacteristic(
    MOTORRIGHTCHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE);


  BLECharacteristic *pMotorLeftChar = pService->createCharacteristic(
    MOTORLEFTCHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE);

  BLECharacteristic *pSteeringChar = pService->createCharacteristic(
    STEERINGCHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE);


  BLECharacteristic *pPowerCharRx = pService->createCharacteristic(
    POWERCHAR_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE);

  pPowerCharTx = pService->createCharacteristic(
    POWERCHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  pMotorRightChar->setCallbacks(new MotorRightCallbacks());
  pMotorLeftChar->setCallbacks(new MotorLeftCallbacks());
  pSteeringChar->setCallbacks(new SteeringCallbacks());
  pPowerCharRx->setCallbacks(new PowerCallbacks());
  pPowerCharTx->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

int count = 0;

void loop() {
  // Get vesc values via uart and send them via ble
  if (vesc.getVescValues()) {
    send_vesc_values();
  }
  delay(3);
}