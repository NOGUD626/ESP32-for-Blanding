#include <PubSubClient.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFi.h>
#include "NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include <ESP32_BME280_I2C.h> 
ESP32_BME280_I2C bme280i2c(0x76, 22, 21, 400000); //address, SCK, SDA, frequency


NineAxesMotion mySensor;                 //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 40;          //To stream at 25Hz without using additional timers (time period(ms) =1000/frequency(Hz))
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream

// Pub/Sub
const char* mqttHost = "192.168.13.2"; // MQTTのIPかホスト名
const int mqttPort = 1883;       // MQTTのポート
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const char* topic = "topic";     // 送信先のトピック名
char* payload;                   // 送信するデータ


void setup() //This code is executed once
{
  WiFiManager wifiManager;
  if (!wifiManager.startConfigPortal("OnDemandAP")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  IPAddress ipadr = WiFi.localIP();
  Serial.println("connected(^^)");
  Serial.println("local ip");
  Serial.println(ipadr);
  Serial.println(WiFi.SSID());
  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();
  updateSensorData = true;
  uint8_t t_sb = 0; //stanby 0.5ms
  uint8_t filter = 4; //IIR filter = 16
  uint8_t osrs_t = 2; //OverSampling Temperature x2
  uint8_t osrs_p = 5; //OverSampling Pressure x16
  uint8_t osrs_h = 1; //OverSampling Humidity x1
  uint8_t Mode = 3; //Normal mode
  bme280i2c.ESP32_BME280_I2C_Init(t_sb, filter, osrs_t, osrs_p, osrs_h, Mode);
  Serial.println();
  Serial.println("Default accelerometer configuration settings...");
  Serial.print("Range: ");
  Serial.println(mySensor.readAccelRange());
  Serial.print("Bandwidth: ");
  Serial.println(mySensor.readAccelBandwidth());
  Serial.print("Power Mode: ");
  Serial.println(mySensor.readAccelPowerMode());
  Serial.println("Streaming in ...");	//Countdown
  Serial.print("3...");
  delay(1000);	//Wait for a second
  Serial.print("2...");
  delay(1000);	//Wait for a second
  Serial.println("1...");
  delay(1000);	//Wait for a second
}

void loop() //This code is looped forever
{
  if (updateSensorData)  //Keep the updating of data as a separate task
  {
     // 送信処理 topic, payloadは適宜
    payload = "payload";
    mqttClient.publish(topic, payload);
    delay(90); 
        
    mySensor.updateAccel();        //Update the Accelerometer data
    mySensor.updateEuler();        //Update the Euler data 
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGravAccel();    //Update the Gravity Acceleration data
    mySensor.updateCalibStatus();  //Update the Calibration Status
    updateSensorData = false;
  }
  if ((millis() - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = millis();

    Serial.print("Time: ");
    Serial.print(lastStreamTime);
    Serial.print("ms ");
    float temperature = (float)bme280i2c.Read_Temperature();
    float humidity = (float)bme280i2c.Read_Humidity();
    uint16_t pressure = (uint16_t)round(bme280i2c.Read_Pressure());
    char temp_c[10], hum_c[10], pres_c[10];
    sprintf(temp_c, "%2d ℃", temperature);
    sprintf(hum_c, "%2d ％", humidity);
    sprintf(pres_c, "%4d hPa", pressure);
    //Serial.println("-----------------------");
    //Serial.print("Temperature = "); Serial.println(temp_c);
    //Serial.print("Humidity = "); Serial.println(hum_c);
    //Serial.print("Pressure = "); Serial.println(pres_c);
    //Serial.println("-----------------------");
    //Serial.flush();
    //Serial.print(" aX: ");
    //Serial.print(mySensor.readAccelerometer(X_AXIS)); //Accelerometer X-Axis data
    //Serial.print("m/s2 ");

    //Serial.print(" aY: ");
    //Serial.print(mySensor.readAccelerometer(Y_AXIS));  //Accelerometer Y-Axis data
    //Serial.print("m/s2 ");

    //Serial.print(" aZ: ");
    //Serial.print(mySensor.readAccelerometer(Z_AXIS));  //Accelerometer Z-Axis data
    //Serial.print("m/s2 ");
    String json = "{";
    json +=  "\"devices\": \"ESP32_Ripro01\"";
    json += ",";
    json +=  "\"type\": \"Linear_Acceleration\"";
    json += ",";
    json += "\"Lx\":";
    json += mySensor.readLinearAcceleration(X_AXIS);
    json += ",";
    json += "\"Ly\":";
    json += mySensor.readLinearAcceleration(Y_AXIS);
    json += ",";
    json += "\"Lz\":";
    json += mySensor.readLinearAcceleration(Z_AXIS);
    json += "}";

    String json1 = "{";
    json1 +=  "\"devices\": \"ESP32_Ripro01\"";
    json1 += ",";
    json1 +=  "\"type\": \"Accelerometer\"";
    json1 += ",";
    json1 += "\"ax\":";
    json1 += mySensor.readAccelerometer(X_AXIS);
    json1 += ",";
    json1 += "\"ay\":";
    json1 += mySensor.readAccelerometer(Y_AXIS);
    json1 += ",";
    json1 += "\"az\":";
    json1 += mySensor.readAccelerometer(Z_AXIS);
    json1 += "}";

    String json2 = "{";
    json2 +=  "\"devices\": \"ESP32_Ripro01\"";
    json2 += ",";
    json2 +=  "\"type\": \"Gravity_Acceleration\"";
    json2 += ",";
    json2 += "\"gx\":";
    json2 += mySensor.readGravAcceleration(X_AXIS);
    json2 += ",";
    json2 += "\"gy\":";
    json2 += mySensor.readGravAcceleration(Y_AXIS);
    json2 += ",";
    json2 += "\"gz\":";
    json2 += mySensor.readGravAcceleration(Z_AXIS);
    json2 += "}";

    String json3 = "{";
    json3 +=  "\"devices\": \"ESP32_Ripro01\"";
    json3 += ",";
    json3 +=  "\"type\": \"Weather_Sensor\"";
    json3 += ",";
    json3 += "\"T\":";
    json3 += temperature;
    json3 += ",";
    json3 += "\"H\":";
    json3 += humidity;
    json3 += ",";
    json3 += "\"P\":";
    json3 += pressure;
    json3 += "}";

    String json4 = "{";
    json4 +=  "\"devices\": \"ESP32_Ripro01\"";
    json4 += ",";
    json4 +=  "\"type\": \"Weather_Sensor\"";
    json4 += ",";
    json4 += "\"EH\":";
    json4 += mySensor.readEulerHeading();
    json4 += ",";
    json4 += "\"ER\":";
    json4 += mySensor.readEulerRoll();
    json4 += ",";
    json4 += "\"EP\":";
    json4 += mySensor.readEulerPitch();
    json4 += "}";
    //Serial.print(json3);
    //Serial.print(" lX: ");
    //Serial.print(mySensor.readLinearAcceleration(X_AXIS)); //Linear Acceleration X-Axis data
    //Serial.print("m/s2 ");

    //Serial.print(" lY: ");
    //Serial.print(mySensor.readLinearAcceleration(Y_AXIS));  //Linear Acceleration Y-Axis data
    //Serial.print("m/s2 ");

    //Serial.print(" lZ: ");
    //Serial.print(mySensor.readLinearAcceleration(Z_AXIS));  //Linear Acceleration Z-Axis data
    //Serial.print("m/s2 ");

    //Serial.print("      gX: ");
    //Serial.print(mySensor.readGravAcceleration(X_AXIS)); //Gravity Acceleration X-Axis data
    //Serial.print("m/s2 ");

    //Serial.print(" gY: ");
    //Serial.print(mySensor.readGravAcceleration(Y_AXIS));  //Gravity Acceleration Y-Axis data
    //Serial.print("m/s2 ");

    //Serial.print(" gZ: ");
    //Serial.print(mySensor.readGravAcceleration(Z_AXIS));  //Gravity Acceleration Z-Axis data
    //Serial.print("m/s2 ");

    //Serial.print("      C: ");
    //Serial.print(mySensor.readAccelCalibStatus());  //Accelerometer Calibration Status (0 - 3)

    Serial.println();
    // MQTT
    if ( ! mqttClient.connected() ) {
      mqttClient.setServer(mqttHost, mqttPort);
        while( ! mqttClient.connected() ) {
            Serial.println("Connecting to MQTT...");
            String clientId = "ESP32-";
            if ( mqttClient.connect(clientId.c_str()) ) {
                Serial.println("connected"); 
            }
            delay(3000);
            randomSeed(micros());
    }
      }
     mqttClient.loop();
     char jsonStr[200];
     mqttClient.publish(topic,"START");
     json.toCharArray(jsonStr,200);
     mqttClient.publish(topic, jsonStr);
     json1.toCharArray(jsonStr,200);
     mqttClient.publish(topic, jsonStr);
     json2.toCharArray(jsonStr,200);
     mqttClient.publish(topic, jsonStr);
     json3.toCharArray(jsonStr,200);
     mqttClient.publish(topic, jsonStr);
     json4.toCharArray(jsonStr,200);
     mqttClient.publish(topic, jsonStr);
     mqttClient.publish(topic,"END");
    updateSensorData = true;
  }
}
