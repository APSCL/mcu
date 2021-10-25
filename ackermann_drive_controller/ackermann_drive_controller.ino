#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

#define DEBUG 0

#define MAX_TURN_DEG 26
#define MAX_TURN_RAD MAX_TURN_DEG * PI / 180
#define WHEEL_BASE 342
#define WHEEL_BASE_METERS 0.342
#define HALF_WHEEL_BASE WHEEL_BASE/2
#define HALF_WHEEL_BASE_METERS 0.171
#define TRACK_WIDTH 191
#define HALF_TRACK_WIDTH_M 0.0955
#define MIN_TURN_RADIUS_METERS WHEEL_BASE_METERS / tan(MAX_TURN_RAD)

// Pin Descriptions
int servo_left_pin = 23;
int servo_right_pin = 22;
int motor_enable_pin = 21;
int motor_fwd_pin = 19;
int motor_bck_pin = 18;
int encoder_a_pin = 26;
int encoder_b_pin = 25;

Servo left_steer, right_steer;

unsigned long last_read;
unsigned long last_count;
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
  
  left_steer.attach(servo_left_pin);
  right_steer.attach(servo_right_pin);
  pinMode(motor_enable_pin, OUTPUT);
  pinMode(motor_fwd_pin, OUTPUT);
  pinMode(motor_bck_pin, OUTPUT);

  left_steer.writeMicroseconds(1500);
  right_steer.writeMicroseconds(1500);
  analogWrite(motor_enable_pin, 0);

  pinMode(encoder_a_pin, INPUT);
  pinMode(encoder_b_pin, INPUT);
  encoder.attachFullQuad(encoder_a_pin, encoder_b_pin);

  /* PID Setup */
  motor_pid.SetMode(AUTOMATIC);
}

void loop() {
  if(Serial.available() >= 8){
    float turn, velocity, angle;
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
    turn = u.f;

    u.b[0] = drive_msg[7];
    u.b[1] = drive_msg[6];
    u.b[2] = drive_msg[5];
    u.b[3] = drive_msg[4];
    velocity = u.f;
        
    /* CLIPPING */
    /*if(turn > MAX_TURN_RAD)        turn = MAX_TURN_RAD;
    else if (turn < -MAX_TURN_RAD) turn = -MAX_TURN_RAD;*/
    
    if(velocity > 1)        velocity = 1;
    else if (velocity < -1) velocity = -1;

    /* STEERING */
    float theta, turn_radius, theta_in, theta_out, angle_left, angle_right;

    if(turn == 0){
      angle_left = angle_right = HALF_PI;
    }
    else{
      /* Turn Scaling */
      // theta = turn;
  
      /* Ackermann Steering Geometry */
      //turn_radius = WHEEL_BASE / tan(theta);
      turn_radius = abs(velocity / turn);

      //if(turn_radius < MIN_TURN_RADIUS_METERS) turn_radius = MIN_TURN_RADIUS_METERS;
     
        theta_in = atan2(WHEEL_BASE_METERS, turn_radius - HALF_TRACK_WIDTH_M);
        theta_out = atan2(WHEEL_BASE_METERS, turn_radius + HALF_TRACK_WIDTH_M);  
      
      
  
      if(turn > 0){
        angle_left = theta_in + HALF_PI;
        angle_right = theta_out + HALF_PI;
      }
      else{
        angle_left = HALF_PI - theta_out;
        angle_right = HALF_PI - theta_in;
      }
    }

    /* Steering Servo Control */
    int pwm_left = round(2000 * angle_left / PI + 500);
    int pwm_right = round(2000 * angle_right / PI + 500);
    left_steer.writeMicroseconds(pwm_left);
    right_steer.writeMicroseconds(pwm_right);

    /* MOTOR*/
    motor_set = abs(velocity) * 120;  // 120 is from experimental maximum rps
    
    if(velocity > 0){
      digitalWrite(motor_fwd_pin, HIGH);
      digitalWrite(motor_bck_pin, LOW);
    }
    else if(velocity < 0){
      digitalWrite(motor_fwd_pin, LOW);
      digitalWrite(motor_bck_pin, HIGH);
    }
    else {
      digitalWrite(motor_fwd_pin, HIGH);
      digitalWrite(motor_bck_pin, HIGH);
    }

    #if DEBUG
    printf("TURN = %f\nVELO = %f\n", turn, velocity);
    printf("ANGLE_L = %f\nANGLE_R = %f\n\n", angle_left, angle_right);
    #endif
  }

  /* Encoder & PID */
  long interval;
  long velo;
  long time_now = millis();
  if((interval = time_now - last_read) >= 10){
    long cur_count = encoder.getCount();
    long delta = encoder.getCount() - last_count;
    velo = delta * 1000 / interval / 64;
    #if DEBUG    
    Serial.print(velo);
    Serial.print(" ");
    Serial.println(motor_set);
    #endif
    last_count = cur_count;
    last_read = time_now;

    motor_encoder = (double)abs(velo);
    motor_pid.Compute();
    if(motor_pwm == 0)  analogWrite(motor_enable_pin, 255);
    else                analogWrite(motor_enable_pin, motor_pwm);
  }
}
