#include <OneWire.h>
#include <DallasTemperature.h>

const int ONE_WIRE_BUS = 2;

=
const float TEMPERATURE_THRESHOLD = 3.0;


float initialTemperature = 0.0;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass the oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// Function to initialize the temperature sensor
void initializeTemperatureSensor() {

  sensors.begin();
  
  sensors.requestTemperatures();
  initialTemperature = sensors.getTempCByIndex(0);
}

float readTemperature() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

// Function to check if the temperature has increased by the threshold
bool isTemperatureIncreased(float currentTemperature) {
  return (currentTemperature - initialTemperature) >= TEMPERATURE_THRESHOLD;
}

//Will add more parts, such as if statemnt saying what happens whne temprature exceeds threshold,ng snesor, etc.