 
#define TINY_GSM_MODEM_SIM800


#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

//Constants
#define DHTPIN 10     // what pin we're connected to
#define DHTTYPE DHT11   // DHT 11  (AM2302)


// Your GPRS credentials
const char apn[]  = "gpinternet";
const char user[] = "";
const char pass[] = "";

SoftwareSerial SerialAT(5, 2, false, 256); // RX, TX

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
//MQTT CERDITIONALS
const char* broker = "mqtt.snifferiot.xyz";


#define redPin  14                //rgb red pin
#define greenPin  12              //rgb green pin
#define bluePin  13               //rgb blue pin
#define BuzPin  16                 // Buzzer connected to digital pin 04
#define batteryPin  4
unsigned int MethaneValue = 0;           //Initiating Gas Sensor value

float R0; //Define variable for R0
unsigned int durationCount = 0;

//reconnect attempt after diconection of broker
unsigned long previousMillisLoop = 0;
unsigned long previousMillisMQTT = 0;
unsigned long previousMillisBuzzer = 0;
unsigned long previousMillisActivation = 0;
unsigned long previousMillisHB = 0;



//thresh check
const byte normalVal = 100;
const byte closeThres = 200;
const int thresVal = 300;

bool buzzerState = LOW;
const byte activation_addr = 0;
const byte R0_stat_addr = 9;
const byte R0_val_addr = 10;
char activation;

String deviceID;
//Topics
String topicUpdate;
String topicStatus;
String topicData;
String topicHB;

byte mqtt_fail_counter = 0;

void setup()
{
  pin_init();
  rgb(0, 0, 1);     //Blue for Device has Powered
  Serial.begin(115200);       // Set console baud rate
  delay(10);

  // Set GSM module baud rate
  SerialAT.begin(115200);
  dht.begin();          //begin DHT11
  EEPROM.begin(512);  //begin EEPROM
  delay(3000);

  byte value = EEPROM.read(activation_addr);
  Serial.print(activation_addr);
  Serial.print("\t");
  Serial.print(value, DEC);
  Serial.println();

  //Checking Activation
  activation = char(EEPROM.read(activation_addr));
  if (activation != 'A') {
    //Write character for Inactivity
    EEPROM.write(activation_addr, 'I');
    //Store data to EEPROM
    EEPROM.commit();
    Serial.println(activation);
  }
  else Serial.println(activation);

  //Checking R0
  int R0_stat = EEPROM.read(R0_stat_addr);
  if (R0_stat != 1)
  {
    //delay(21600000);
    calc_R0();
    Serial.println("R0 Calculated!");
    store_R0();
    Serial.println("R0 Stored!");
  }
  else
  {
    Serial.println("R0 already exists! R0=");
    R0 = EEPROM.read(R0_val_addr);
    Serial.println(R0);
  }

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");

  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);
  String imei = modem.getIMEI();
  Serial.print("IMEI: ");
  Serial.println(imei);

  deviceID = String(modem.getIMEI());

  topicUpdate = "snf/" + deviceID + "/update";

  topicStatus = "snf/" + deviceID + "/status";

  topicData = "snf/" + deviceID + "/data";

  topicHB = "snf/" + deviceID + "/hb";

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(" fail");
    while (true);
    //    Serial.println("Can't connect to network!!!  Reseting everything....");
    //    ESP.restart();
  }
  Serial.println(" OK");

  //White for GPRS Conected
  rgb(1, 1, 1);

  Serial.print("Connecting to ");
  Serial.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(" fail");
    while (true);
    //    Serial.println("Can't connect to GPRS!!!  Reseting everything....");
    //    ESP.restart();
  }
  Serial.println(" OK");

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

  //Green for Device being operational
  rgb(0, 1, 0);

  ESP.wdtDisable();
  ESP.wdtEnable(WDTO_8S);

}

boolean mqttConnect() {
  Serial.print("Connecting to ");
  Serial.print(broker);
  String clientId = "SnifferIoTClient-";
  clientId += String(random(0xffff), HEX);

  if (!modem.isGprsConnected()) { 
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(" fail");
    while (true);
    //    Serial.println("Can't connect to GPRS!!!  Reseting everything....");
    //    ESP.restart();
  } }
  
  if (!mqtt.connect(clientId.c_str(), "SnifferIoT", "AllahIs1"))
  {
    //    Serial.println(" Fail");

    Serial.println(mqtt.state());

    mqtt_fail_counter = mqtt_fail_counter + 1;
    if (mqtt_fail_counter >= 3)
    {
      Serial.println("MQTT failed to connect 3 times a row!!!  Reseting everything....");
      while (true);
      //      ESP.restart();
    }
    return false;
  }
  mqtt_fail_counter = 0;
  Serial.println(" OK");

  mqtt.subscribe(topicStatus.c_str(), 1);
  return mqtt.connected();
}

void loop() {
  ESP.wdtFeed();

  unsigned long currentMillisLoop = millis();

  if (currentMillisLoop - previousMillisLoop >= 2000L)
  {

    previousMillisLoop = currentMillisLoop;
    //Serial.println("at connected function");
    Serial.flush();
    SerialAT.flush();
    if (mqtt.connected())
    {
      //Serial.println("at loop");
      mqtt.loop();
    }
    else
    {
      yield();
      mqttConnect();
    }

    //if (!modem.isGprsConnected()) { mqttConnect();}
  }


  MethaneValue = get_ppm();       //Reads the analaog value from the methane sensor's A0 pin

  if(MethaneValue >= 10000)
  {
    MethaneValue = 9999;
  }


  // MQ4 Sensore
  if (MethaneValue > thresVal ) give_alarm();
  else
  {
    digitalWrite(BuzPin, LOW);
    rgb(0, 1, 0);
  }

  //  Serial.println("Chechkpoint 2 passed!");

  if (activation == 'I')
  {
    unsigned long currentMillisActivation = millis();
    if (currentMillisActivation - previousMillisActivation >= 10000L)
    {

      previousMillisActivation = currentMillisActivation;
      mqtt.publish(topicUpdate.c_str(), "activate" );
      Serial.println(activation);
    }
  }

  else if (activation == 'A')
  {
    SendHeartBeat();
    if (MethaneValue <= normalVal)      SendGasPpm(50000L);
    else if (MethaneValue > normalVal && MethaneValue <= closeThres)      SendGasPpm(30000L);
    else if (MethaneValue > closeThres || MethaneValue >= thresVal)       SendGasPpm(10000L);
  }

}


void mqttCallback(char* topic, byte * payload, unsigned int len) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.write(payload, len);
  Serial.println();

  // Only proceed if incoming message's topic matches
  if (String(topic) == topicStatus.c_str())
  {
    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1')
    {
      //Write character for activity
      EEPROM.write(activation_addr, 'A');
      //Store data to EEPROM
      EEPROM.commit();
      Serial.println(activation);
      while (true);
    }
    else if ((char)payload[0] == '0')
    {
      //Write character for Inactivity
      EEPROM.write(activation_addr, 'I');
      //Store data to EEPROM
      EEPROM.commit();
      Serial.println(activation);
      while (true);
    }
  }
}

//Sending Gas MQTT
int SendGasPpm(int intervalMQTT)
{

  char MethaneValChar[5], SignalStrnthChar[2];
  int hum;                    //Stores humidity value
  int temp;                   //Stores temperature value
  int prevhum;
  int prevtemp;
  byte signalStrength;
  char message[20];
  char batteryState;
  unsigned long currentMillisMQTT = millis();

  // checks if 10 delay is over
  if (currentMillisMQTT - previousMillisMQTT >= intervalMQTT)
  {

    sprintf(MethaneValChar, "%04d", MethaneValue); //convert gas value to string
    hum = dht.readHumidity();       //Read data and store it to variables hum and temp
    temp = dht.readTemperature();

    if(!isnan(temp))
    {
      prevtemp = temp;
      prevhum = hum;
    }
    if(isnan(temp))
    {
      temp = prevtemp;
      hum = prevhum;
    }
    
    signalStrength = modem.getSignalQuality();
    sprintf(SignalStrnthChar, "%02d", signalStrength);

    if (digitalRead(batteryPin) == 0)    batteryState = 'A';
    else    batteryState = 'B';
    // save last time sensor reading
    previousMillisMQTT = currentMillisMQTT;

    //JSON in String Form
    String payload = String(MethaneValChar) + String(temp) + String(hum) + String(SignalStrnthChar) + String(batteryState);
    //String payload = String(MethaneValChar) + String(SignalStrnthChar) + String(batteryState);
    //String payload = String(MethaneValChar) + "25" + "64" + String(SignalStrnthChar) + String(batteryState);
    payload.toCharArray(message, (payload.length() + 1));
    mqtt.publish(topicData.c_str(), message );

    Serial.println(message);
    Serial.println(activation);
    Serial.print("Heap: ");
    Serial.println(ESP.getFreeHeap());
  }
  return 0;
}

void SendHeartBeat() {
  unsigned long currentMillisHB = millis();

  // checks if 10 delay is over
  if (currentMillisHB - previousMillisHB >= 10000L)
  {
    ++durationCount;
    // save last time sensor reading
    previousMillisHB = currentMillisHB;
    mqtt.publish(topicHB.c_str(), "1" );
    Serial.println(MethaneValue);
    Serial.println(durationCount);
  }
}

void calc_R0()
{
  float sensor_volt; //Define variable for sensor voltage
  float RS_air; //Define variable for sensor resistance
  float sensorValue; //Define variable for analog readings

  for (int x = 0 ; x < 100 ; x++)
  {
    sensorValue = sensorValue + analogRead(A0); //Add analog values of sensor 500 times
    Serial.print(".");
    delay(200);
  }
  sensorValue = sensorValue / 100.0; //Take average of readings
  sensor_volt = sensorValue * (5.0 / 1023.0); //Convert average to voltage
  RS_air = ((5.0 * 1) / sensor_volt) - 1; //Calculate RS in fresh air  //RS = [(VC x RL) / VRL] - RL
  R0 = RS_air / 4.4; //Calculate R0, for fresh air:  RS / R0 = 4.4 ppm

  Serial.println();
  Serial.print("R0 = ");
  Serial.println(R0); //Display value of R0
}

void store_R0()
{
  EEPROM.write(R0_stat_addr, 1);
  EEPROM.write(R0_val_addr, R0);
  EEPROM.commit();
  delay(100);
}

int get_ppm()
{
  float m = -0.65488, b = 0.96464;
  float sensor_volt; //Define variable for sensor voltage
  float RS_gas; //Define variable for sensor resistance
  float ratio; //Define variable for ratio
  float sensorValue = analogRead(A0); //Read analog values of sensor
  sensor_volt = sensorValue * (5.0 / 1023.0); //Convert analog values to voltage
  RS_gas = ((5.0 * 1) / sensor_volt) - 1; //Get value of RS in a gas //RS = [(VC x RL) / VRL] - RL
  ratio = RS_gas / R0;   // Get ratio RS_gas/RS_air

  double ppm_log = (log10(ratio) - b) / m; //Get ppm value in linear scale according to the the ratio value
  double ppm = pow(10, ppm_log); //Convert ppm value to log scale
  return ppm;
}

void rgb(int r, int g, int b)
{
  if (r == 1)  digitalWrite(redPin, LOW);
  else  digitalWrite(redPin, HIGH);

  if (g == 1)  digitalWrite(greenPin, LOW);
  else  digitalWrite(greenPin, HIGH);

  if (b == 1)  digitalWrite(bluePin, LOW);
  else  digitalWrite(bluePin, HIGH);
}

void give_alarm()
{
  unsigned long currentMillisBuzzer = millis();
  if (currentMillisBuzzer - previousMillisBuzzer >= 500L)
  {
    previousMillisBuzzer = currentMillisBuzzer;           // save the last time you triggeres the Buzzer
    if (buzzerState == LOW)         // if the Buzzer is off turn it on and vice-versa:
    {
      buzzerState = HIGH;
      digitalWrite(BuzPin, buzzerState);
      rgb(1, 0, 0);
    }
    else
    {
      buzzerState = LOW;
      digitalWrite(BuzPin, buzzerState);
      rgb(0, 0, 0);
    }
  }
}

void pin_init()
{
  pinMode(BuzPin, OUTPUT);      // Sets the digital pin as output
  pinMode(batteryPin, INPUT);      // Sets the digital pin as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

