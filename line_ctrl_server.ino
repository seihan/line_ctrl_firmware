#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "3a39152a-6371-4730-8e24-31be298cf059"
#define MOTORRIGHTCHAR_UUID "74454618-2b9a-4c9a-bc20-b351dc7bd269" // MotorRight
#define MOTORLEFTCHAR_UUID  "bf3e592d-063b-4b25-884e-5814640054e9" // MotorLeft
#define POWERCHAR_UUID      "6cc05bc7-d9da-4b6e-9bfa-65e6c0b5b9d3" // MotorMain

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
  M1_RPWM = 27,
  M1_LPWM = 25,
  M2_RPWM = 16,
  M2_LPWM = 17,
};

String mRight = "";
String mLeft = "";
String mPower = "";

// Just keep current movement settings in global variables
struct state {
  bool forward;
  int16_t speed;
} motors[2];

void set_speed_forward_m1(int16_t new_speed)
{
  motors[0].forward = true;
  motors[0].speed = new_speed;
  ledcWrite(M1_BWD_CH, 0);
  ledcWrite(M1_FWD_CH, new_speed);
}

void set_speed_backward_m1(int16_t new_speed)
{
  motors[0].forward = false;
  motors[0].speed = new_speed;
  ledcWrite(M1_FWD_CH, 0);
  ledcWrite(M1_BWD_CH, new_speed);
}

void set_speed_forward_m2(int16_t new_speed)
{

  motors[1].forward = true;
  motors[1].speed = new_speed;
  ledcWrite(M2_BWD_CH, 0);
  ledcWrite(M2_FWD_CH, new_speed);
}

void set_speed_backward_m2(int16_t new_speed)
{
  motors[1].forward = false;
  motors[1].speed = new_speed;
  ledcWrite(M2_FWD_CH, 0);
  ledcWrite(M2_BWD_CH, new_speed);
}


// Full STOP
void full_stop()
{
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

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("disconnected");
      Serial.println("Bluetooth start advertising...");
      BLEAdvertising *pAdvertising = pServer->getAdvertising();
      pAdvertising->start();

    }
};

class MotorRightCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue().c_str();
      if (value.length() > 0) {
        Serial.print("Motor right ");
        Serial.println(value);
        int intValue = value.toInt();
        if (intValue < 0) {
          set_speed_backward_m1(-intValue);
        } else {
          set_speed_forward_m1(intValue);
        }
      }
    }
};

class MotorLeftCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue().c_str();
      if (value.length() > 0) {
        Serial.print("Motor left ");
        Serial.println(value);
        int intValue = value.toInt();
        if (intValue < 0) {
          set_speed_backward_m2(-intValue);
        } else {
          set_speed_forward_m2(intValue);
        }
      }
    }
};

class PowerCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue().c_str();
      if (value.length() > 0) {       
        Serial.print("Motor power ");
        Serial.println(value);
        int intValue = value.toInt();
      }
    }
};

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup motor driver...");
  // motor driver
  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH);
  ledcSetup(M1_FWD_CH, FREQ, RESOLUTION);
  ledcSetup(M1_FWD_CH, FREQ, RESOLUTION);
  ledcSetup(M2_BWD_CH, FREQ, RESOLUTION);
  ledcSetup(M2_BWD_CH, FREQ, RESOLUTION);
  ledcAttachPin(M1_RPWM, M1_FWD_CH);
  ledcAttachPin(M1_LPWM, M1_BWD_CH);
  ledcAttachPin(M2_RPWM, M2_FWD_CH);
  ledcAttachPin(M2_LPWM, M2_BWD_CH);

  full_stop();

  // bluetooth

  Serial.println("Setup bluetooth service...");
  BLEDevice::init("LineCtrl");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  Serial.println("Setup bluetooth characteristics...");
  BLECharacteristic *pMotorRightChar = pService->createCharacteristic(
                                         MOTORRIGHTCHAR_UUID,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );


  BLECharacteristic *pMotorLeftChar = pService->createCharacteristic(
                                        MOTORLEFTCHAR_UUID,
                                        BLECharacteristic::PROPERTY_WRITE
                                      );

  BLECharacteristic *pPowerChar = pService->createCharacteristic(
                                    POWERCHAR_UUID,
                                    BLECharacteristic::PROPERTY_WRITE
                                  );

  Serial.println("Setup bluetooth callbacks...");
  pMotorRightChar->setCallbacks(new MotorRightCallbacks());
  pMotorLeftChar->setCallbacks(new MotorLeftCallbacks());
  pPowerChar->setCallbacks(new PowerCallbacks());

  pService->start();

  Serial.println("Bluetooth start advertising...");
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  Serial.println("Setup ready.");
}

void loop()
{
  // Read input
  int cmd = 0;
  if (Serial.available()) {
    cmd = Serial.read();
  }

  int16_t speed;

  // Adapt movement
  // q / a / z - motor1 speed up / stop / speed down
  // w / s / x - motor2 speed up / stop / speed down
  // 1 - reverse motor 1 direction
  // 2 - reverse motor 2 direction
  switch (cmd) {
    case 'q':
      speed = min(0xff, motors[0].speed + 10);
      Serial.print("Motor 1 Go Faster: ");
      Serial.println(speed);
      set_speed_forward_m1(speed);
      break;
    case 'y':
      speed = max(0, motors[0].speed - 10);
      Serial.print("Motor 1 Go Slower: ");
      Serial.println(speed);
      set_speed_forward_m1(speed);
      break;
    case 'a':
      Serial.println("Motor 1 Full STOP!");
      full_stop_m1();
      break;
    case 'w':
      speed = min(0xff, motors[0].speed + 10);
      Serial.print("Motor 1 Backward Go Faster: ");
      Serial.println(speed);
      set_speed_backward_m1(speed);
      break;
    case 'x':
      speed = max(0, motors[0].speed - 10);
      Serial.print("Motor 1 Backward Go Slower: ");
      Serial.println(speed);
      set_speed_backward_m1(speed);
      break;
    case 'e':
      speed = min(0xff, motors[0].speed + 10);
      Serial.print("Motor 2 Go Faster: ");
      Serial.print("Motor 2 Engage! Going ");
      Serial.println(motors[1].forward ? "FORWARD" : "BACKWARDS");

      Serial.println(speed);
      set_speed_forward_m2(speed);
      break;
    case 'd':
      Serial.println("Motor 2 Full STOP!");
      full_stop_m2();
      break;
    case 'c':
      speed = max(0, motors[0].speed - 10);
      Serial.print("Motor 1 Go Slower: ");
      Serial.println(speed);
      set_speed_forward_m2(speed);
      break;
    case 'r':
      speed = min(0xff, motors[0].speed + 10);
      Serial.print("Motor 2 Backward Go Faster: ");
      Serial.println(speed);
      set_speed_backward_m2(speed);
      break;
    case 'v':
      speed = max(0, motors[0].speed - 10);
      Serial.print("Motor 2 Backward Go Slower: ");
      Serial.println(speed);
      set_speed_backward_m2(speed);
      break;
    case 'f':
      Serial.println("Motor 1 Full Speed Forward!");
      motors[0].forward = false;
      set_speed_forward_m1(255);
      break;
    case 'g':
      Serial.println("Motor 1 Full Speed Backwards!");
      motors[1].forward = false;
      set_speed_backward_m1(255);
      break;
    case 'h':
      Serial.println("Motor 2 Full Speed Forward!");
      motors[0].forward = false;
      set_speed_forward_m2(255);
      break;
    case 'j':
      Serial.println("Motor 2 Full Speed Backwards!");
      motors[1].forward = false;
      set_speed_backward_m2(255);
      break;
    default:
      break;
  }
}
