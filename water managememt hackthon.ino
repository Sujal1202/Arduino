#include <DallasTemperature.h>

// Data wire is conntec to the Arduino digital pin 4
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

#include <WiFi.h>
#include <ThingsBoard.h>
float leakage;
#define WIFI_AP "Sujal Vaish"
#define WIFI_PASSWORD "sujal@00"

#define TOKEN "tsJ25sxTBnUI4zl0k2wk"

#define SENSOR  27
#define SENSOR1  12
#define SENSOR2  13
int relay=14;
int trigPin1 = 25;
int echoPin1 = 26;
//int trigPin2 = 32;
//int echoPin2 = 33;
long duration, distance, tank1;
int turbidity;
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
float calibrationFactor = 7.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;

long currentMillis1 = 0;
long previousMillis1 = 0;
int interval1 = 1000;
float calibrationFactor1 = 7.5;
volatile byte pulseCount1;
byte pulse1Sec1 = 0;
float flowRate1;

long currentMillis2 = 0;
long previousMillis2 = 0;
int interval2 = 1000;
float calibrationFactor2 = 7.5;
volatile byte pulseCount2;
byte pulse1Sec2 = 0;
float flowRate2;

float flowMilliLitres;
float totalMilliLitres;

float flowMilliLitres1;
float totalMilliLitres1;

float flowMilliLitres2;
float totalMilliLitres2;
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}
void IRAM_ATTR pulseCounter1()
{
  pulseCount1++;
}

void IRAM_ATTR pulseCounter2()
{
  pulseCount2++;
}

char thingsboardServer[] = "demo.thingsboard.io";

WiFiClient wifiClient;

ThingsBoard tb(wifiClient);

int status = WL_IDLE_STATUS;
unsigned long lastSend;

void setup()
{
  Serial.begin(115200);
  pinMode(SENSOR, INPUT_PULLUP);
  pinMode(SENSOR1, INPUT_PULLUP);
  pinMode(SENSOR2, INPUT_PULLUP);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  sensors.begin();
  //pinMode(trigPin2, OUTPUT);
  //pinMode(echoPin2, INPUT);
  pinMode(relay, OUTPUT);

  
  delay(10);
  InitWiFi();
  lastSend = 0;

  pulseCount1 = 0;
  flowRate1= 0.0;
  flowMilliLitres1 = 0;
  totalMilliLitres1 = 0;
  previousMillis1 = 0;

  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  pulseCount2 = 0;
  flowRate2= 0.0;
  flowMilliLitres2 = 0;
  totalMilliLitres2 = 0;
  previousMillis2 = 0;

  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(SENSOR1), pulseCounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(SENSOR2), pulseCounter2, RISING);

}

void loop()
{
  sensors.requestTemperatures(); 
  int sensorValue = analogRead(34);
  float voltage = sensorValue * (3.3 / 4095.0);
  int turbidity = map(sensorValue, 0,4095, 0, 100);
  SonarSensor(trigPin1, echoPin1);
  tank1 = distance;
  //SonarSensor(trigPin2, echoPin2);
 // tank2 = distance;
  
  if (tank1 > 6 && tank1<22)
   {
    digitalWrite(relay, LOW);
  } 
  else 
  {
    digitalWrite(relay, HIGH);
  }
  

  if ( !tb.connected() ) {
    reconnect();
  }

  if ( millis() - lastSend > 1000 ) { // Update and send only after 1 seconds
    water();
    lastSend = millis();

  }
  tb.loop();
}
  void water()
  {
  currentMillis = millis();
  currentMillis1 = millis();
  currentMillis2 = millis();
  if (currentMillis - previousMillis > interval)
  {
    pulse1Sec = pulseCount;
    pulseCount = 0;
    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();

    flowMilliLitres = flowRate / 60;
    totalMilliLitres += flowMilliLitres;
     }
  if (currentMillis1 - previousMillis1 > interval1)
  {
    pulse1Sec1 = pulseCount1;
    pulseCount1 = 0;
    flowRate1 = ((1000.0 / (millis() - previousMillis1)) * pulse1Sec1) / calibrationFactor1;
    previousMillis1 = millis();

    flowMilliLitres1 = flowRate1 / 60;
    totalMilliLitres1 += flowMilliLitres1;
  }

  if (currentMillis2 - previousMillis2 > interval2)
  {
    pulse1Sec2 = pulseCount2;
    pulseCount2 = 0;
    flowRate2 = ((1000.0 / (millis() - previousMillis2)) * pulse1Sec2) / calibrationFactor2;
    previousMillis2 = millis();

    flowMilliLitres2 = flowRate2 / 60;
    totalMilliLitres2 += flowMilliLitres2;
    Serial.print("Output Liquid Quantity2: ");
    Serial.println(totalMilliLitres2);
    float leakage=flowRate-flowRate1;
    }
    Serial.print(tank1);
    Serial.print(" - ");
    //Serial.print(tank2);
    //Serial.print(" - ");


  tb.sendTelemetryFloat("flowrate1", flowRate);
  tb.sendTelemetryFloat("flowrate2", flowRate1);
  tb.sendTelemetryFloat("flowrate3", flowRate2);
  tb.sendTelemetryFloat("totalwater1", totalMilliLitres);
  tb.sendTelemetryFloat("totalwater2", totalMilliLitres1);
  tb.sendTelemetryFloat("totalwater3", totalMilliLitres2);
  tb.sendTelemetryFloat("tank1", tank1);
  tb.sendTelemetryFloat("turbidity", turbidity);
  tb.sendTelemetryFloat("leakage", leakage);
  tb.sendTelemetryFloat("temp", sensors.getTempCByIndex(0));
}

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}


void reconnect() {
  // Loop until we're reconnected
  while (!tb.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println( "[DONE]" );
    } else {
      Serial.print( "[FAILED]" );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}
void SonarSensor(int trigPin,int echoPin)
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance = duration*0.034/2;
}
