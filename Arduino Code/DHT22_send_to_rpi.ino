#include "DHT.h"

#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define LED 13

int maxHum = 60;
int maxTemp = 40;

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(9600); 
  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(20000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.print(t);
  Serial.print(' ');
  Serial.println(h);
  digitalWrite(LED,digitalRead(LED)^1);
  
}
