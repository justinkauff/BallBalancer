#include <Servo.h>


/*
Pseudocode THESE WILL NEED TO BE ADOPTED TO TWO DEGREES (x,y)

Three Parts

Global Setup (variables wanted to be used everywhere)
i.e. float x;

float [x_home,y_home] = centered position/setpoint
float i = 0; error integration
float t1;
float t2;
float d; // err derivative
float e_last;
float kp = calculated value;
float ki = calculated value;
float kd = calculated value;
float u0 = the signal that flattens the plate

Init
i.e. x = 5;
1) setup up sensor
2) setup motors
3) setup serial
4) t1 = timer();
y = get_sensor_val();
e_last = [x_home,y_home]-[x,y];


Loop
i.e. x = x + 1;

t2 = timer();
dt = t2-t1;
t1 = timer();
[x,y] = get_sensor_val;


2) e = [x_home,y_home]-[x,y];
3) i += e * dt;
4) d = (e-e_last)/dt;
5) e_last = e

u = kp*e + ki*i + kd*d; PID SIGNAL
motor_command = u0 - u;
execute(motor_command);

*/

// pin setup
#define XP A1   // X+
#define XM A0   // X-
#define YP A3   // Y+
#define YM A2   // Y-

// Servo Declaration
Servo servoX, servoY;

// plate calibrations
const int X_MIN = 40;
const int X_MAX = 955;
const int Y_MIN = 61;
const int Y_MAX = 927;

// Output coordinate range (mm)
const int W = 187;
const int H = 141;

// Touch detect thresholds (tune if needed)
const int Z_MIN = 80;     // minimum "pressure" to count as touch
const int Z_MAX = 900;    // ignore crazy values

// GLOBAL VARIABLES

// coordinates of plate center
float x_home = 97;
float y_home = 74;

// current coordinates of the ball
int x_ball = 0;
int y_ball = 0;

// intergation/derviative needs
float i_x = 0;
float i_y = 0;

float d_x = 0;
float d_y = 0;

// derivative filtering
float d_x_raw = 0;
float d_y_raw = 0;

// time trackers
unsigned long t1;
unsigned long t2;
float dt = 0;

// errors
float e_last_x = 0;
float e_last_y = 0;
float e_x = 0;
float e_y = 0;

// tuning parameters
float kp = 2.0;
float ki = 0.167;
float kd = 1.375;

// the signals that flatten the plate
float u0_x = 1650;
float u0_y = 1350;
float alpha = 0.9; // derivative filtering as 1.375 is a little high but the minimum for performace chars

// placeholder signals
float u_x;
float u_y;

// placeholders for writeMicroseconds()
float motor_command_x;
float motor_command_y;

// for first error signal
bool initialized = false;

// PLATE READING METHODS

int readXraw() {
  // Drive X axis, read YP
  pinMode(XP, OUTPUT); digitalWrite(XP, HIGH);
  pinMode(XM, OUTPUT); digitalWrite(XM, LOW);
  pinMode(YP, INPUT);
  pinMode(YM, INPUT);
  delayMicroseconds(200);
  return analogRead(YP);
}

int readYraw() {
  // Drive Y axis, read XP
  pinMode(YP, OUTPUT); digitalWrite(YP, HIGH);
  pinMode(YM, OUTPUT); digitalWrite(YM, LOW);
  pinMode(XP, INPUT);
  pinMode(XM, INPUT);
  delayMicroseconds(200);
  return analogRead(XP);
}

// Simple pressure estimate for 4-wire panels.
// Not physically perfect, but works well enough to detect "touch vs not touch".
int readZraw() {
  // Measure resistance-ish by driving one corner and reading the opposite sheet.
  pinMode(XP, OUTPUT); digitalWrite(XP, LOW);
  pinMode(YM, OUTPUT); digitalWrite(YM, HIGH);
  pinMode(XM, INPUT);
  pinMode(YP, INPUT);
  delayMicroseconds(200);

  int z1 = analogRead(XM);
  int z2 = analogRead(YP);
  int z  = abs(z2 - z1);
  return z;
}

int mapAndClamp(int v, int inMax, int inMin, int outMin, int outMax) {
  // Note: inMax -> inMin because your axes are inverted (top-left high)
  long out = (long)(v - inMax) * (outMax - outMin) / (long)(inMin - inMax) + outMin;
  if (out < outMin) out = outMin;
  if (out > outMax) out = outMax;
  return (int)out;
}

// tests if valid press, sets x and y position of ball
bool get_sensor_val(int &x, int &y) {
  const int N = 5;

  long xSum = 0;
  long ySum = 0;
  int goodSamples = 0;

  for (int i = 0; i < N; i++) {
    int z = readZraw();

    if (z > Z_MIN && z < Z_MAX) {
      int xRaw = readXraw();
      int yRaw = readYraw();

      xSum += xRaw;
      ySum += yRaw;
      goodSamples++;
    }

    delayMicroseconds(500);
  }

  if (goodSamples < 3) {
    return false;
  }

  int xRawAvg = xSum / goodSamples;
  int yRawAvg = ySum / goodSamples;

  x = mapAndClamp(xRawAvg, X_MAX, X_MIN, 0, W);
  y = mapAndClamp(yRawAvg, Y_MAX, Y_MIN, 0, H);

  // Serial.print("X_mm:");
  // Serial.print(x);
  // Serial.print(",");

  // Serial.print("Y_mm:");
  // Serial.println(y);
  Serial.print(millis());   // time (ms)
  Serial.print(",");
  Serial.print(x);          // x_mm
  Serial.print(",");
  Serial.println(y);        // y_mm

  return true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  servoX.attach(9);
  servoY.attach(11);

  servoX.writeMicroseconds(u0_x);
  servoY.writeMicroseconds(u0_y);


  t1 = micros();
  

}

void loop() {
  // put your main code here, to run repeatedly:
  t2 = micros();
  dt = (t2 - t1) / 1000000.0; // convert to seconds;
  t1 = t2;

  if (dt <= 0) return;

  if(!get_sensor_val(x_ball,y_ball)){
    return;
  }
  e_x = - x_home + x_ball;
  e_y =  y_home - y_ball;

  // trying to fix the random servo spaz near setpoint

  float deadband = 2.0;
  bool near_x = abs(e_x) < deadband;
  bool near_y = abs(e_y) < deadband;

  if(!initialized){
    e_last_x = e_x;
    e_last_y = e_y;
    initialized = true;
    return;
  }

  i_x += e_x * dt;
  d_x_raw = (e_x-e_last_x)/dt;
  e_last_x = e_x;

  i_y += e_y * dt;
  d_y_raw = (e_y-e_last_y)/dt;
  e_last_y = e_y;


  i_x = constrain(i_x, -50, 50);
  i_y = constrain(i_y, -50, 50);

  // derivative filtering, might make servos less jittery
  d_x = alpha * d_x + (1 - alpha) * d_x_raw;
  d_y = alpha * d_y + (1 - alpha) * d_y_raw;
  // limit derivative spike
  d_x = constrain(d_x, -80, 80);
  d_y = constrain(d_y, -80, 80);

  if(near_x) d_x = 0;
  if(near_y) d_y = 0;

  u_x = kp*e_x + ki*i_x + kd*d_x;
  u_y = kp*e_y + ki*i_y + kd*d_y;

  motor_command_x = u0_x + u_x;
  motor_command_y = u0_y + u_y;

  motor_command_x = constrain(motor_command_x, 1000, 2000);
  motor_command_y = constrain(motor_command_y, 1000, 2000);

  servoX.writeMicroseconds((int)motor_command_x);
  servoY.writeMicroseconds((int)motor_command_y);  

}
