// TODO:
// IMU - Needs to be initialized, periodically poll sensor, store Z-axis angle in variable, take initial value as 0 deg print out to serial monitor, look into kalman filters for reducing sensor noise.
// Drive Motors - Need need to add PID with IMU.
// Servo Motor - Needs to be initialized, turn 180 deg clockwise once at start to dump reactants, wait 1 sec to finish dumping, turn back 180 deg counter-clockwise to return to upright position, set speed to 100% for now, tune later.

// Included libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SD.h>
#include <SPI.h>

// Define drive motor pins
#define left_pwm1 9
#define left_pwm2 10
#define right_pwm1 11
#define right_pwm2 12

// Define the PWM pins for the stir bar motor
#define stirPin1 5
#define stirPin2 6

#define ONE_WIRE_BUS A1 // pin for the DS18B20 data line

// Define CS pin for SD card
#define chip_select 4

OneWire oneWire(ONE_WIRE_BUS);       // create a OneWire instance to communicate with the senso
DallasTemperature sensors(&oneWire); // pass oneWire reference to Dallas Temperature sensor

// Define files
File root;
File nextFile;
File dataFile;
String fileName;

bool isFileNew = false; // Checks for new file

// Temperature threshold
const float tempDiff = 3;

// Time limit in milliseconds
const unsigned long tLim = 12000;

// Initialize run count for SD card file
int runCount;
int checkRun;

// Define accelerometer variables
float zAngle;         // z-axis angle
float zAngleFiltered; // Filtered z-axis angle

// variables to store temperature
float temperatureC;
float initTemp;

float data[4]; // Data array

// KALMAN FILTER variables
float k;         // kalman gain
float x_k;       // Filtered temperature
float p_k;       // Initial error covariance
float x_k_minus; // Predicted next state estimate
float p_k_minus; // Predicted error covariance for the next state

// Process noise and measurement noise
float q; // Process noise covariance
float r; // Measurement noise covariance

// Keeping track of time
unsigned long currTime;
unsigned long startTime;

void drive_forward(int speed) // Drive function
{
  // left wheel
  digitalWrite(left_pwm1, HIGH);
  analogWrite(left_pwm2, speed);

  // right wheel
  digitalWrite(right_pwm1, HIGH);
  analogWrite(right_pwm2, speed);
}

void stop_driving() // Stop function
{
  // left wheel
  digitalWrite(left_pwm1, HIGH);
  analogWrite(left_pwm2, 0);

  // right wheel
  digitalWrite(right_pwm1, HIGH);
  analogWrite(right_pwm2, 0);
}

void printer(bool serialTrue, unsigned long millisTime, float outputs[4]) // Output function
{
  if (serialTrue)
  {
    Serial.print(millisTime);

    for (int i = 0; i < sizeof(outputs); i++)
    {
      Serial.print(",");
      Serial.print(outputs[i]);
    }

    Serial.println("");
  }
  else
  {
    dataFile.print(millisTime);

    for (int i = 0; i < sizeof(outputs); i++)
    {
      dataFile.print(",");
      dataFile.print(outputs[i]);
    }

    dataFile.println("");
  }
}

void setup() // Setup (executes once)
{
  // Get time at start
  startTime = millis();

  Serial.begin(9600); // start serial communication (adjust baud rate as needed)
  Serial.println("Initalised at 9600 bps");

  // Initialize Kalman filter parameters
  x_k = 0.0; // Initial state estimate
  p_k = 1.0; // Initial error covariance
  q = 0.01;  // Process noise covariance
  r = 0.1;   // Measurement noise covariance

  // Initialize SD Card
  Serial.println("SD card is initializing...");

  if (!SD.begin(chip_select))
  {
    Serial.println("SD card initialization failed.");
    while (1)
      ; // Halts and waits if failure
  }

  root = SD.open("/"); // Open SD root directory
  checkRun = 0;

  while (true)
  {
    nextFile = root.openNextFile();

    if (nextFile)
    { // Increment with each existing file
      checkRun++;
    }
    else
    {
      nextFile.close();
      break;
    }
  }

  root.close();

  runCount = checkRun + 1;

  Serial.println("Success! SD card initialized.");
  Serial.println("Time,Temperature,Filtered Temperature,z-angle,Filtered z-angle"); // Data header
  
  // Initialize the stir motor pins as outputs
  pinMode(stirPin1, OUTPUT);
  pinMode(stirPin2, OUTPUT);

  // Set the stir initial speed to 80%
  analogWrite(stirPin1, 204);  // 80% of 255
  digitalWrite(stirPin2, LOW); // for fast decay

  sensors.begin();                       // initialize the DS18B20 sensor
  sensors.requestTemperatures();         // request temperature from all devices on the bus
  initTemp = sensors.getTempCByIndex(0); // get temperature in Celsius

  // Servo acctuation goes here

  // Setting drive motors to output mode
  pinMode(left_pwm1, OUTPUT);
  pinMode(left_pwm2, OUTPUT);
  pinMode(right_pwm1, OUTPUT);
  pinMode(right_pwm2, OUTPUT);

  // Start drive motors completely stopped
  analogWrite(left_pwm1, 0);
  digitalWrite(left_pwm2, LOW);
  analogWrite(right_pwm1, 0);
  digitalWrite(right_pwm2, LOW);
}

void loop() // Loop (main loop)
{
  sensors.requestTemperatures();             // request temperature from all devices on the bus
  temperatureC = sensors.getTempCByIndex(0); // get temperature in Celsius

  // Kalman filter prediction
  x_k_minus = x_k;     // Predicted next state estimate
  p_k_minus = p_k + q; // Predicted error covariance for the next state

  // Kalman filter update

  /* Kalman gain: calculated based on the predicted error covariance
  and the measurement noise covariance,,, used to update the
  state estimate (x_k) and error covariance (p_k) */
  k = p_k_minus / (p_k_minus + r); // kalman gain

  // comparison with actual temp reading
  x_k = x_k_minus + k * (temperatureC - x_k_minus); // Updated state estimate
  p_k = (1 - k) * p_k_minus;                        // Updated error covariance

  // Recieve IMU data here

  fileName = "Run_" + String(runCount) + ".csv";
  dataFile = SD.open(fileName, FILE_WRITE);

  if (dataFile)
  {
    // Writes header if it's a new file
    if (!isFileNew)
    {
      dataFile.println("Time,Temperature,Filtered Temperature,z-Angle,Filtered z-Angle");
      isFileNew = true;
    }

    // Obtain current time in seconds
    currTime = millis();

    // Update data array
    data[0] = temperatureC;
    data[1] = x_k;
    data[2] = zAngle;
    data[3] = zAngleFiltered;

    // Write variable data to the file in CSV format
    printer(false, currTime, data);

    dataFile.close();
    isFileNew = false; // Reset for next file
  }
  else
  {
    Serial.println("Error! Cannot open file for writing.");
  }

  // Print variable data to serial in CSV format
  printer(true, currTime, data);

  drive_forward(204); // 80% speed is 204

  // Add PID here

  if (((x_k - initTemp) > tempDiff) || ((currTime - startTime) > tLim))
  {
    stop_driving();
  }
}
