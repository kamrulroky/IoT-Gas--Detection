#define FREQUENCY    80                  // valid 80, 160
#define BuzPin  16                 // Buzzer connected to digital pin 04

volatile uint8_t buzzFlag = 0;
volatile uint8_t serialFlag = 0;

void timer0_ISR (void) {

  timer0_write(ESP.getCycleCount() + 80000000L); // 80MHz == 1sec
  buzzFlag = 1;
  serialFlag = 1;
}
#include "ESP8266WiFi.h"
extern "C" {
#include "user_interface.h"
}

int gas_sensor = A0; //Sensor pin
float m = -0.318; //Slope 
float b = 1.133; //Y-Intercept
float R0 = 192.12; //Sensor Resistance in fresh air from previous code
const int thresVal = 100;
bool buzzerState = LOW;

void setup() {
  Serial.begin(9600); //Baud rate
  pinMode(gas_sensor, INPUT); //Set gas sensor as input
  pinMode(BuzPin, OUTPUT);
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
  system_update_cpu_freq(FREQUENCY);
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(timer0_ISR);
  timer0_write(ESP.getCycleCount() + 80000000L); // 80MHz == 1sec
  interrupts();
}

void loop() {
  float sensor_volt; //Define variable for sensor voltage
  float RS_gas; //Define variable for sensor resistance
  float ratio; //Define variable for ratio
  float sensorValue = analogRead(gas_sensor); //Read analog values of sensor
  sensor_volt = sensorValue * (5.0 / 1023.0); //Convert analog values to voltage
  RS_gas = ((5.0 * 9.78) / sensor_volt) - 9.78; //Get value of RS in a gas
  ratio = RS_gas / R0;   // Get ratio RS_gas/RS_air

  double ppm_log = (log10(ratio) - b) / m; //Get ppm value in linear scale according to the the ratio value
  double ppm = pow(10, ppm_log); //Convert ppm value to log scale

    if(ppm >= 10000)
  {
    ppm = 9999;
  }

    if (ppm > thresVal )
    give_alarm();
  else
  {
    digitalWrite(BuzPin, LOW);
  }
  

  if (serialFlag == 1)
     {
      Serial.print("PPM methane = ");
      Serial.println(ppm);
      serialFlag = 0;
     }
}

void give_alarm()
{
  if(buzzFlag == 1)
  {
    digitalWrite(BuzPin, !(digitalRead(BuzPin)));
    buzzFlag = 0;
  }
}
