#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// Search and replace "82/garasje/control/doorpulse" with the topic you want to control it with
// payload "1" sends the pulse directly to the door opener
// payload "2" sends the pulse only if the door has a valid open position
// payload "3" opens the pulse only if the door has a valid closed position

// Search and replace "82/garasje/status/doorpulse" to read the outout to the door opener

// Search and replace "82/garasje/status/state" to read the state of the door. 
// 1 Closed, 2 Opening, 3 Closing, 4 Open

// Search and replace "82/garasje/doorposition" to read the door position in %
// Search and replace "82/garasje/encCounter" to read the counter of the encoder, nice when setting up first time
// Topics "82/garasje/status/calibrated", "82/garasje/status/calibratedTop", "82/garasje/status/calibratedBottom"
// are just for indication that the door has first been opened fully and the closed fully 
// in that sequence. And then the "calibrated" is set to "1". This is reset after power cycling.



#ifndef STASSID
#define STASSID "<SSID>" //SSID for WiFi
#define STAPSK  "<PASSWORD>" //Password for Wifi
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;

//Must be set up
char* mqttServer = "<SERVER>";
int mqttPort = 1883;
char* mqttUser = "<USER>";
char* mqttPassword = "<PASSWORD>";
char* clientname = "<CLIENTNAME>";

//Number of counts from open to closed
int doorCounts = 4950;

int topPosition = 97;
int bottomPosition = 3;

int encApin = 12; //If encoder counts negative when 
int encBpin = 14; //opening swap pins for encApin and encBpin

int topPin = 8;
int bottomPin = 9;

//Variables below here are only for internal processes
int encAval = 0;
int encBval = 0;

int encAprev = 0;
int encBprev = 0;
int topPinPrev = 0;
int bottomPinPrev = 0;

int encCounter = 10000;
int doorPosition = 0;
int doorPositionPrev = 0;

int calibrated = 0;

char msg_out[20];
int doorPulseLength = 2000;
int doorPulseStart = 0;
int doorPulseState = 0;
int pulsePin = 13;
int doorState = 0;

int topCal = 0;
int bottomCal = 0;
int lastTransmit = 0;

WiFiClient espClient;
PubSubClient client(espClient);


void callback(char* topic, byte* payload, unsigned int length) {
  String payloadS;
  String topicS1;
  String topicS = String((char*)topic);
  int beskjed;
  Serial.print("Message arrived [");
  Serial.println(topic);
  for (int j = 0; j < 25; j++) {
    topicS1 += (char)topic[j];
  }
  for (int i = 0; i < length; i++) {
    payloadS += (char)payload[i];
  }

  if (topicS == "82/garasje/control/doorpulse" and payloadS == "1")
  {
    Serial.println("Opererer port");
    doorPulseStart = millis();
    doorPulseState = 1;
    client.publish("82/garasje/status/doorpulse", "1");
    digitalWrite(pulsePin, LOW);
  }
  else if (topicS == "82/garasje/control/doorpulse" and payloadS == "2" and doorState == 4)
  {
    Serial.println("Lukker port");
    doorPulseStart = millis();
    doorPulseState = 1;
    client.publish("82/garasje/status/doorpulse", "1");
    digitalWrite(pulsePin, LOW);
  }
  else if (topicS == "82/garasje/control/doorpulse" and payloadS == "3" and doorState == 1)
  {
    Serial.println("Åpner port");
    doorPulseStart = millis();
    doorPulseState = 1;
    client.publish("82/garasje/status/doorpulse", "1");
    digitalWrite(pulsePin, LOW);
  }
}

int doorstatus(int encC, int steps)
{
  int doorPercentage = 0;
  encC = encC - 10000;
  doorPercentage = 10000 / steps * encC;
  return doorPercentage / 100;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starter program");

  pinMode(encApin, INPUT);
  pinMode(encBpin, INPUT);
  pinMode(pulsePin, OUTPUT);
  digitalWrite(pulsePin, HIGH);

  //Sett opp interrupt
  //attachInterrupt(encApin, isr, RISING);

  //Sett opp WiFi
  Serial.print("Kobler til WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //Sett opp MQTT
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    Serial.println("Kobler til MQTT boot...");

    if (client.connect(clientname, mqttUser, mqttPassword )) {

      Serial.println("connected");

      delay(500);
      client.subscribe("82/garasje/control/doorpulse");
    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }
  client.setCallback(callback);

  // OTA
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.setHostname(clientname);
  // OTA FERDIG
  ArduinoOTA.begin();

}

void loop() {
  ArduinoOTA.handle();
  client.loop();

  calibration();
  connectionChecks();
  pulseReset();

  encAval = digitalRead(encApin);
  encBval = digitalRead(encBpin);
  encReading();


  publishMQTT();
}

void pulseReset() {
  if (millis() - doorPulseLength > doorPulseStart and doorPulseState == 1) {
    doorPulseState = 0;
    client.publish("82/garasje/status/doorpulse", "0");
    digitalWrite(pulsePin, HIGH);
  }
}

void encReading() { 
  if (encAval != encAprev and encAval == 1)
  {
    turning();
    doorPosition = doorstatus(encCounter, doorCounts);
  }

  if (encAval != encAprev)
  {
    encAprev = encAval;
  }
}

void turning() {
  if (encBval == 0 and encCounter < 15000)
  {
    encCounter = encCounter + 1;
  }
  if (encBval == 1 and encCounter > 9999)
  {
    encCounter = encCounter - 1;
  }
}

void connectionChecks() {
    if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFitilkobling mistet");
  }

  if (!client.connected()) {
    Serial.println("Kobler til MQTT igjen...");

    if (client.connect(clientname, mqttUser, mqttPassword )) {
      Serial.println("connected");
      client.subscribe("82/garasje/control/doorpulse");

    }
  }
}

void publishMQTT() {
  if (lastTransmit + 2000 < millis() or doorPosition != doorPositionPrev)
  {
    Serial.println(doorPosition);
    sprintf(msg_out, "%d", doorPosition);
    client.publish("82/garasje/doorposition", msg_out);
    sprintf(msg_out, "%d", encCounter);
    client.publish("82/garasje/encCounter", msg_out);
    if (doorPosition < bottomPosition) {
      doorState = 1; //Lukket
    }
    else if (doorPosition > doorPositionPrev and doorPosition >= bottomPosition and doorPosition < topPosition) {
      doorState = 2; // Åpner
    }
    else if (doorPosition < doorPositionPrev and doorPosition >= bottomPosition and doorPosition < topPosition) {
      doorState = 3; // Lukker
    }
    else if (doorPosition >= topPosition) {
      doorState = 4; //Åpen
    }
    sprintf(msg_out, "%d", doorState);
    client.publish("82/garasje/status/state", msg_out);
    sprintf(msg_out, "%d", calibrated);
    client.publish("82/garasje/status/calibrated", msg_out);
    sprintf(msg_out, "%d", topCal);
    client.publish("82/garasje/status/calibratedTop", msg_out);
    sprintf(msg_out, "%d", bottomCal);
    client.publish("82/garasje/status/calibratedBottom", msg_out);
    lastTransmit = millis();
    doorPositionPrev = doorPosition;
  }
}

void calibration() {
    if (topCal == 0 and encCounter > doorCounts + 10000 - 500) {
    topCal = 1;
  }
  else if (bottomCal == 0 and encCounter < 10500 and topCal == 1) {
    bottomCal = 1;
  }
  else if (bottomCal == 1 and topCal == 1) {
    calibrated = 1;
  }
}
