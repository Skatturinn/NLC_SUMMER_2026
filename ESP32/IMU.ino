#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define MUX_ADDR 0x70
#define BNO_ADDR 0x28



struct __attribute__((__packed__)) SensorData {
  double quat[4];       // w, x, y, z
  float linAcc[3];      // x, y, z
  float gravity[3];     // x, y, z
  uint8_t calibration;  // System, Gyro, Accel, Mag (2 bits each)
};

// Main packet struct
struct __attribute__((__packed__)) ImuDataPacket {
  uint8_t header[2];       // Sync bytes: 0xAA, 0xBB
  uint32_t timestamp;      // ESP32 system uptime
  
  SensorData sensors[2];   // Index 0: Base, Index 1: Arm, Index 2: Wrist
  
  uint8_t checksum;        
};

class ImuTrackedSensor {
private:
  const char* name;
  uint8_t channel;
  Adafruit_BNO055* bno;

  void selectMuxChannel() {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delayMicroseconds(500); 
  }

public:
  ImuTrackedSensor(const char* imuName, uint8_t muxChannel, Adafruit_BNO055* sharedBNO) {
    name = imuName;
    channel = muxChannel;
    bno = sharedBNO;
  }

  bool begin() {
    selectMuxChannel();
    if (!bno->begin()) {

      Serial.print("MSG,ERROR: Could not find ");
      Serial.println(name);
      return false;
    }
    bno->setExtCrystalUse(true);
    return true;
  }

  // Fills a SensorData struct directly
  void getSample(SensorData &data) {
    selectMuxChannel();
    
    imu::Quaternion qRaw = bno->getQuat();
    imu::Vector<3> linAccRaw = bno->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gravRaw = bno->getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    
    uint8_t sys, gyro, accel, mag = 0;
    bno->getCalibration(&sys, &gyro, &accel, &mag);

    // Populate Quaternions
    data.quat[0] = qRaw.w();
    data.quat[1] = qRaw.x();
    data.quat[2] = qRaw.y();
    data.quat[3] = qRaw.z();

    // Populate Linear Acceleration
    data.linAcc[0] = linAccRaw.x();
    data.linAcc[1] = linAccRaw.y();
    data.linAcc[2] = linAccRaw.z();

    // Populate Gravity
    data.gravity[0] = gravRaw.x();
    data.gravity[1] = gravRaw.y();
    data.gravity[2] = gravRaw.z();

    // Pack calibration into a single byte to save bandwidth
    // Format: [Sys:2][Gyro:2][Accel:2][Mag:2]
    data.calibration = (sys << 6) | (gyro << 4) | (accel << 2) | mag;
  }
};
Adafruit_BNO055 sharedBNO = Adafruit_BNO055(55, BNO_ADDR);

// FIXED: Added array size to declare an array of 3 sensors
ImuTrackedSensor imuSensors[2] = { // \squarebrackets{3}
  // ImuTrackedSensor("BASE",  0, &sharedBNO),
  ImuTrackedSensor("ARM",   7, &sharedBNO),
  ImuTrackedSensor("WRIST", 4, &sharedBNO)
};

ImuDataPacket packet;
unsigned long lastTxTime = 0;
const unsigned long txIntervalMs = 20; 

uint8_t calculateChecksum(uint8_t* data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i]; // \squarebrackets{i}
  }
  return crc; //  bit loss check but not if correct order of bytes
}

void setup() {
  Serial.begin(500000); 
  while(!Serial);

  Wire.begin(21, 22);       
  Wire.setClock(100000);    

  // FIXED: Array indices added for the header bytes
  packet.header[0] = 0xAA; // \squarebrackets{0}
  packet.header[1] = 0xBB; // \squarebrackets{1}

  for (int i = 0; i < 2; i++) {
    if (!imuSensors[i].begin()) { // \squarebrackets{i}
      while (1) { delay(10); } 
    }
    delay(100);
  }
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTxTime >= txIntervalMs) {
    lastTxTime = currentTime;
    packet.timestamp = currentTime;

    // Loop through sensors to fill their respective structs
    for (int i = 0; i < 2; i++) {
      imuSensors[i].getSample(packet.sensors[i]);
    }
// Calculate checksum skipping the 2 header bytes and the 1 checksum byte
    packet.checksum = calculateChecksum(((uint8_t*)&packet) + 2, sizeof(ImuDataPacket) - 3);
    
    // Write out the entire packet strictly as binary
    Serial.write((uint8_t*)&packet, sizeof(ImuDataPacket));
  }
}