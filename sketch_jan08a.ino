#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>

unsigned long prev_read_time = millis();
String gelenVeri;
String tt, var1;
int ind1, ind2;
int servoAngle=0;
int airq = 0;


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

Servo myservo;  // create servo object to control a servo
static const int servoPin = 13;



BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;

#define I2C_SDA 26                      //New CLK for BME280
#define I2C_SCL 27
#define BME280 ADDRESS 0x76
#define SEALEVELPRESSURE_HPA (1013.25)


float temperature, humidity, pressure;
Adafruit_BME280 bme;

TwoWire I2CBME = TwoWire(1);



const int LED = 32;
const int mq135 = 33;
int sensorValue = 0;




class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
}

          if (rxValue.find("on") != -1) {
            digitalWrite(LED, LOW);
            rxValue.clear();
          }
          else if (rxValue.find("off") != -1) {
            digitalWrite(LED, HIGH);
            rxValue.clear();
          }
        if(rxValue[0]==101){
          gelenVeri = rxValue.c_str();
          ind1 = gelenVeri.indexOf(',');
          tt = gelenVeri.substring(0,ind1);
          ind2 = gelenVeri.indexOf(',', ind1 + 1);
          var1 = gelenVeri.substring(ind1 + 1, ind2);
          servoAngle = var1.toInt();
          myservo.write(servoAngle);
          Serial.println(servoAngle);

          
          
        }
      }
    }
};


void setup() {
  Serial.begin(115200);
  Wire.begin();
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);
  myservo.attach(servoPin); 

  bme.begin(0x76, &I2CBME);
  pinMode(LED, OUTPUT);
  pinMode(LED, LOW);
  pinMode(mq135, OUTPUT);



    // Create the BLE Device
  BLEDevice::init("HomeAutomation"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_READ
                    );

  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

}

void loop() {
  
 if (deviceConnected) {
    if (millis() - prev_read_time > 2000) {

      digitalWrite(mq135, HIGH);
      sensorValue = analogRead(mq135);
      if(sensorValue > 1900){
        airq = 1;
      }
      else{
        airq = 0;
      }
      digitalWrite(mq135, LOW);


      Serial.println(sensorValue);
     

      temperature = bme.readTemperature();
      static char temp1[6];
      dtostrf(temperature, 2, 0, temp1);
      //Serial.println(temp1);


      humidity = bme.readHumidity();
      static char humi1[8];
      dtostrf(humidity, 2, 0, humi1);



      pressure = bme.readPressure();
      static char press1[8];
      dtostrf(pressure, 4, 0, press1);

     
      char txString[32];
      sprintf(txString, "%s,%s,%s,%d", temp1, humi1, press1,airq);
      Serial.println(txString);

      pCharacteristic->setValue(txString);

      pCharacteristic->notify(); // Send the value to the app!

     
      prev_read_time = millis();
    }

  }

}
