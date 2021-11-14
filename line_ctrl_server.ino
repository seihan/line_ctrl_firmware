#include <ssl_client.h>
#include <WiFiClientSecure.h>

#include <analogWrite.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "3a39152a-6371-4730-8e24-31be298cf059"
#define MOTORRIGHTCHAR_UUID "74454618-2b9a-4c9a-bc20-b351dc7bd269" // MotorRight
#define MOTORLEFTCHAR_UUID  "bf3e592d-063b-4b25-884e-5814640054e9" // MotorLeft
#define POWERCHAR_UUID      "6cc05bc7-d9da-4b6e-9bfa-65e6c0b5b9d3" // MotorMain

// L298N - H-Bridge DC Motor driver:
//   ENA <- PIN D6
//   IN1 <- Pin D7
//   IN2 <- Pin D8
//   ENB <- PIN D9
//   IN3 <- PIN D10
//   IN4 <- PIN D11

#include <Wire.h>

enum Pins {
  M1_ENABLE = 6,
  M1_FWD = 7,
  M1_RWD = 8,
  M2_ENABLE = 9,
  M2_FWD = 10,
  M2_RWD = 11,
};

char motorRight[8] = {}; //for incoming value

String mRight = "";
String mLeft = "";
String mPower = "";

// Just keep current movement settings in global variables
struct state {
  bool forward;
  int16_t speed;
} motors[2];

void set_speed(int16_t new_speed)
{
  set_speed_m1(new_speed);
  set_speed_m2(new_speed);
}

void set_speed_m1(int16_t new_speed)
{
  motors[0].speed = new_speed;
  //analogWrite(M1_ENABLE, new_speed);
}

void set_speed_m2(int16_t new_speed)
{
  motors[1].speed = new_speed;
  //analogWrite(M2_ENABLE, new_speed);
}

// Full STOP
void full_stop()
{
  full_stop_m1();
  full_stop_m2();
}

void full_stop_m1() {
  digitalWrite(M1_FWD, LOW);
  digitalWrite(M1_RWD, LOW);

  set_speed_m1(0);

  motors[0].forward = true;
}

void full_stop_m2() {
  digitalWrite(M2_FWD, LOW);
  digitalWrite(M2_RWD, LOW);

  set_speed_m2(0);

  motors[1].forward = true;
}

// Reverse the direction
void reverse_m1()
{
  motors[0].forward ^= 0x1;

  // CAUTION: Always pull one output to LOW first! Never set both directions HIGH!
  if (motors[0].forward) {
    digitalWrite(M1_RWD, LOW);
    digitalWrite(M1_FWD, HIGH);
  } else {
    digitalWrite(M1_FWD, LOW);
    digitalWrite(M1_RWD, HIGH);
  }

}

void reverse_m2()
{
  motors[1].forward ^= 0x1;

  // CAUTION: Always pull one output to LOW first! Never set both directions HIGH!
  if (motors[1].forward) {
    digitalWrite(M2_RWD, LOW);
    digitalWrite(M2_FWD, HIGH);
  } else {
    digitalWrite(M2_FWD, LOW);
    digitalWrite(M2_RWD, HIGH);
  }
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
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        for (int i = 0; i < value.length(); i++) {
          mRight += (char)value[i];
        }
        Serial.println("Motor right " + mRight);
        mRight = "";
      }
    }
};

class MotorLeftCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        for (int i = 0; i < value.length(); i++) {
          mLeft += (char)value[i];
        }
        Serial.println("Motor left " + mLeft);
        mLeft = "";
      }
    }
};

class PowerCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        for (int i = 0; i < value.length(); i++) {
          mPower += (char)value[i];
        }
        Serial.println("Motor power " + mPower);
        mPower = "";
      }
    }
};

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup motor driver...");
  // motor driver
  //  pinMode(M1_ENABLE, OUTPUT);
  //  pinMode(M1_FWD, OUTPUT);
  //  pinMode(M1_RWD, OUTPUT);
  //
  //  pinMode(M2_ENABLE, OUTPUT);
  //  pinMode(M2_FWD, OUTPUT);
  //  pinMode(M2_RWD, OUTPUT);

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
                                         //BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );


  BLECharacteristic *pMotorLeftChar = pService->createCharacteristic(
                                        MOTORLEFTCHAR_UUID,
                                        //BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_WRITE
                                      );

  BLECharacteristic *pPowerChar = pService->createCharacteristic(
                                    POWERCHAR_UUID,
                                    //BLECharacteristic::PROPERTY_READ |
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
      set_speed_m1(speed);
      break;
    case 'z':
      speed = max(0, motors[0].speed - 10);
      Serial.print("Motor 1 Go Slower: ");
      Serial.println(speed);
      set_speed_m1(speed);
      break;
    case 'a':
      Serial.println("Motor 1 Full STOP!");
      full_stop_m1();
      break;
    case '1':
      reverse_m1();
      Serial.print("Motor 1 Engage! Going ");
      Serial.println(motors[0].forward ? "FORWARD" : "BACKWARDS");
      break;
    case 'w':
      speed = min(0xff, motors[1].speed + 10);
      Serial.print("Motor 2 Go Faster: ");
      Serial.println(speed);
      set_speed_m2(speed);
      break;
    case 'x':
      speed = max(0, motors[1].speed - 10);
      Serial.print("Motor 2 Go Slower: ");
      Serial.println(speed);
      set_speed_m2(speed);
      break;
    case 's':
      Serial.println("Motor 2 Full STOP!");
      full_stop_m2();
      break;
    case '2':
      reverse_m2();
      Serial.print("Motor 2 Engage! Going ");
      Serial.println(motors[1].forward ? "FORWARD" : "BACKWARDS");
      break;
    case 'f':
      Serial.println("Motor 1 Full Speed Forward!");
      motors[0].forward = false;
      reverse_m1();
      set_speed_m1(255);
      break;
    case 'g':
      Serial.println("Motor 2 Full Speed Forward!");
      motors[1].forward = false;
      reverse_m2();
      set_speed_m2(255);
      break;
    default:
      break;
  }
}
