// library incldues
#include "MPU9250.h"
#include "ESP32Servo.h"
#include <AutoPID.h>
#include <HCSR04.h>
// includes for server communication
#include <WiFi.h>
#include <HTTPClient.h>

// defines for the pins
#define SERVO_PIN_L 1
#define SERVO_PIN_R 2
#define ESC_PIN_L 3
#define ESC_PIN_R 4
#define MPU_PIN_SDA  21    // I2C default SDA GPIO21
#define MPU_PIN_SCL  22    // I2C default SCL GPIO22
#define DIS_PIN_TRIG 5    // pin for the ultrasound distance sensor
#define DIS_PIN_ECHO 6    // pin for the ultrasound distance sensor


// defines for settings
#define CALIBRATE_ACC false
#define CALIBRATE_MAG false
#define SEND_SENSOR_DATA false

// defines for PID settings
#define PID_UPDATE_INTERVAL 1000 	//update intervall in ms
#define ROLL_P .12
#define ROLL_I .0003
#define ROLL_D 0.0
#define ROLL_SET_POINT_START 90
#define YAW_P .12
#define YAW_I .0003
#define YAW_D 0
#define YAW_SET_POINT_START 0
#define HEIGHT_P .12
#define HEIGHT_I .0
#define HEIGHT_D 0
#define HEIGHT_SET_POINT_START 200      // [cm]

Servo servo_l;    // create servo objects to control a servo
Servo servo_r;    // create servo objects to control a servo
Servo esc_l;      // create servo object to send PWM signal to the ESC that controlls the motor
Servo esc_r;      // create servo object to send PWM signal to the ESC that controlls the motor

MPU9250 mpu;      // create a object for the MPU communication

HCSR04 distance_sensor(DIS_PIN_TRIG, DIS_PIN_ECHO); //initialisation class HCSR04 (trig pin , echo pin)

double orientation[3];	// array for the orientaion pitch / roll / yaw
double height;
double control_variables[4];	// array for the controll variables servo_l, servo_r, esc_l, esc_r

// https://r-downing.github.io/AutoPID/#basic-temperature-control PID lib
double roll_pid_output = 0;
double roll_pid_setPoint = ROLL_SET_POINT_START;   // [degree]
AutoPID roll_pid_controller( &orientation[1], &roll_pid_setPoint, &roll_pid_output ,
                             0, 180, ROLL_P, ROLL_I, ROLL_D );

double height_pid_output = 0;
double height_pid_setPoint = HEIGHT_SET_POINT_START;   // [degree]
AutoPID height_pid_controller( &height, &height_pid_output, &height_pid_setPoint ,
                               -90, 90, HEIGHT_P, HEIGHT_I, HEIGHT_D );

// DATA TRANSFER STUFF
const char* ssid = "Vodafone-2D47";
const char* password = "N4cqLeuaE66UbrdJ";

//Your Domain name with URL path or IP address with path
String serverName = "http://192.168.0.117:1880/update-sensor";

// function declarations
void update_height();
void update_orientation();
void update_control_loop();
void update_controls();
void update_sensor_data();
void post_data();

void setup()
{
  Serial.println("setup started");

  // start the serial communication
  Serial.begin(115200);

  // start the I2C communiation for the mpu
  Wire.begin();
  mpu.setup(0x68);

  // calibrate the mpu
  if (CALIBRATE_ACC)
  {
    Serial.println("setup Gyro");
    mpu.calibrateAccelGyro();
  }
  if (CALIBRATE_MAG)
  {
    Serial.println("setup Mag");
    mpu.calibrateMag();
  }

  // initalize servos
  servo_l.attach(SERVO_PIN_L);
  servo_r.attach(SERVO_PIN_R);
  esc_l.attach(ESC_PIN_L, 1000, 2000);
  esc_r.attach(ESC_PIN_R, 1000, 2000);

  // connect to network and server
  if (SEND_SENSOR_DATA)
  {
    WiFi.begin(ssid, password);
    Serial.println("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
  }

  // get the first postion update
  update_orientation();
  update_height();

  // start the PID controller
  roll_pid_controller.setBangBang(10); // TODO check wheter bang makes sense
  roll_pid_controller.setTimeStep(PID_UPDATE_INTERVAL);

  Serial.println("setup finished");
}

void loop() {

  // update the orienation data
  update_orientation();

  // send sensor data to server
  if (SEND_SENSOR_DATA)
  {
    if (WiFi.status() == WL_CONNECTED)
      post_data();
  }

  // use PID controller to compute new control variables
  update_control_loop();

  // update and set the new control variables
  update_controls();
}

void update_height()
{
  //TODO the orientation influences the distance the sensor measures
  // compute the right distance based on the orientation
  height = distance_sensor.dist();
}

void update_orientation()
{
  orientation[0] = mpu.getPitch();
  orientation[1] = mpu.getRoll();
  orientation[2] = mpu.getYaw();
}

void update_controls()
{
  servo_l.write(control_variables[0]);
  servo_r.write(control_variables[1]);
  esc_l.write(control_variables[2]);
  esc_r.write(control_variables[3]);
}

void update_control_loop()
{
  // clear the previous control values
  control_variables[0] = 0;
  control_variables[1] = 0;
  control_variables[2] = 0;
  control_variables[3] = 0;

  
  // height controls
  height_pid_controller.run();
  control_variables[1] += roll_pid_output;  // give power to both motors if too low
  control_variables[2] += roll_pid_output;

  // orientation controls
  // calcultate the new controller values
  roll_pid_controller.run();

  // controll variable logic for the roll
  control_variables[1] += roll_pid_output;	// TODO check signs
  control_variables[2] -= roll_pid_output;

  // TODO add compensation for pitch angle 
}

void post_data()
{
  WiFiClient client;
  HTTPClient http;
  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);
  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  // Data to send with HTTP POST
  String httpRequestData = "value1=" + String(orientation[0]) +
                           "&value2=" + String(orientation[1]) +
                           "&value3=" + String(orientation[2]) +
                           "&value3=" + String(height);
  // Send HTTP POST request
  int httpResponseCode = http.POST(httpRequestData);
};
