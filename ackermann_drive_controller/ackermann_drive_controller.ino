#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

#define DEBUG 0

#define MAX_TURN_DEG 42
#define MAX_TURN_RAD MAX_TURN_DEG * PI / 180
#define MIN_TURN_RADIUS WHEEL_BASE / tan(MAX_TURN_RAD)

#define WHEEL_BASE 0.342
#define HALF_WHEEL_BASE WHEEL_BASE / 2

#define TRACK_WIDTH 0.191
#define HALF_TRACK_WIDTH TRACK_WIDTH / 2

#define INVERSE_STEER 1

// Pin Descriptions
int servo_left_pin = 23;
int servo_right_pin = 22;
int motor_enable_pin = 21;
int motor_fwd_pin = 19;
int motor_bck_pin = 18;
int encoder_a_pin = 26;
int encoder_b_pin = 25;

Servo servo_left, servo_right;

unsigned long count_prev, time_prev;
ESP32Encoder encoder;

double Kp = 8;
double Ki = 7;
double Kd = 0.4;
double motor_encoder = 0;
double motor_pwm = 0;
double motor_set = 0;
PID motor_pid(&motor_encoder, &motor_pwm, &motor_set, Kp, Ki, Kd, DIRECT);

void setup() {
  #if DEBUG
  Serial.begin(115200);
  #endif
  Serial.begin(9600);

  /* Servo Setup */
  servo_left.attach(servo_left_pin);
  servo_right.attach(servo_right_pin);
  servo_left.writeMicroseconds(1500);
  servo_right.writeMicroseconds(1500);

  /* Motor Setup */
  pinMode(motor_enable_pin, OUTPUT);
  pinMode(motor_fwd_pin, OUTPUT);
  pinMode(motor_bck_pin, OUTPUT);
  analogWrite(motor_enable_pin, 0);

  pinMode(encoder_a_pin, INPUT);
  pinMode(encoder_b_pin, INPUT);
  encoder.attachFullQuad(encoder_a_pin, encoder_b_pin);

  /* PID Setup */
  motor_pid.SetMode(AUTOMATIC);
}

void loop() {
  if(Serial.available() >= 8){
    float ang_z, lin_x;
    byte drive_msg[8];
    Serial.readBytes(drive_msg, 8);
    
    union {
      float f;
      byte b[4];
    } u;

    u.b[0] = drive_msg[3];
    u.b[1] = drive_msg[2];
    u.b[2] = drive_msg[1];
    u.b[3] = drive_msg[0];
    ang_z = u.f;

    u.b[0] = drive_msg[7];
    u.b[1] = drive_msg[6];
    u.b[2] = drive_msg[5];
    u.b[3] = drive_msg[4];
    lin_x = u.f;

    #if DEBUG
    printf("ANG_Z = %f LIN_X = %f\n", ang_z, lin_x);
    #endif
    
    if(lin_x > 1)        lin_x = 1;
    else if (lin_x < -1) lin_x = -1;

    /* STEERING */
    float turn_radius, theta_in, theta_out, angle_left = HALF_PI, angle_right = HALF_PI;

    if(ang_z != 0){ 
      /* Ackermann Steering Geometry */
      turn_radius = abs(lin_x / ang_z);

      if(turn_radius < MIN_TURN_RADIUS) turn_radius = MIN_TURN_RADIUS;
     
      theta_in = atan2(WHEEL_BASE, turn_radius - HALF_TRACK_WIDTH);
      theta_out = atan2(WHEEL_BASE, turn_radius + HALF_TRACK_WIDTH);      

    #if !INVERSE_STEER
      if(ang_z > 0){
        angle_left = HALF_PI + theta_in;
        angle_right = HALF_PI + theta_out;
      }
      else{
        angle_left = HALF_PI - theta_out;
        angle_right = HALF_PI - theta_in;
      }
    #else
      if(ang_z < 0){
        angle_left = HALF_PI + theta_in;
        angle_right = HALF_PI + theta_out;
      }
      else{
        angle_left = HALF_PI - theta_out;
        angle_right = HALF_PI - theta_in;
      }
    #endif
    }
    
    /* Steering Servo Control */
    int pwm_left = round(2000 * angle_left / PI + 500);
    int pwm_right = round(2000 * angle_right / PI + 500);
    servo_left.writeMicroseconds(pwm_left);
    servo_right.writeMicroseconds(pwm_right);

    /* Motor Control */
    motor_set = abs(lin_x) * 120;  // 120 is from experimental maximum rps
    
    if(lin_x > 0){
      digitalWrite(motor_fwd_pin, HIGH);
      digitalWrite(motor_bck_pin, LOW);
    }
    else if(lin_x < 0){
      digitalWrite(motor_fwd_pin, LOW);
      digitalWrite(motor_bck_pin, HIGH);
    }
    else {
      digitalWrite(motor_fwd_pin, HIGH);
      digitalWrite(motor_bck_pin, HIGH);
    }
  }

  /* Encoder & PID */
  long time_delta, rps;
  unsigned long time_current = millis();
  if((time_delta = time_current - time_prev) >= 10){
    long count_current = encoder.getCount();
    long count_delta = count_current - count_prev;
    rps = count_delta * 15.625 / time_delta;
    count_prev = count_current;
    time_prev = time_current;

    motor_encoder = (double)abs(rps);
    motor_pid.Compute();
    if(motor_pwm == 0)  analogWrite(motor_enable_pin, 255);
    else                analogWrite(motor_enable_pin, motor_pwm);
  }
}
