#include <EEPROM.h>
#include <PinChangeInt.h>
#include <PID_v1.h>
#include <pidautotuner.h>

#define MATH_ABS(x) ((x) > 0 ? (x) : -(x))

#define SERIAL_BAUD_RATE  (9600)

#define MOTOR_LEFT_ENB_PIN (12)
#define MOTOR_LEFT_PWM_PIN (10)
#define MOTOR_LEFT_ENC_PIN (2)

#define MOTOR_RIGHT_ENB_PIN (13)
#define MOTOR_RIGHT_PWM_PIN (11)
#define MOTOR_RIGHT_ENC_PIN (3)

#define MOTOR_PWM_MIN (0)
#define MOTOR_PWM_MAX (255)
#define MOTOR_PULSE_PER_ROUND (120)
#define MOTOR_TUNE_CYCLES (100)
#define MOTOR_TUNNER_TARGET (3) // round
#define MOTOR_TUNNER_INTERVAL (100) // milliseconds

typedef enum {
  MOTOR_LEFT = (1 << 0),
  MOTOR_RIGHT = (1 << 1),
  MOTOR_ALL = (MOTOR_LEFT | MOTOR_RIGHT)
} MOTOR;

typedef struct motor_pid_t {
  double kp, ki, kd;
} motor_pid;

typedef struct motor_data_t {
  unsigned char status;
  motor_pid motor_left_pid;
  motor_pid motor_right_pid;
} motor_data;

volatile int motor_left_encoder_position = 0;
volatile int motor_right_encoder_position = 0;
motor_data motor_pid_data;
double motor_left_kp, motor_left_ki, motor_left_kd, motor_left_input, motor_left_output, motor_left_setpoint;
double motor_right_kp, motor_right_ki, motor_right_kd, motor_right_input, motor_right_output, motor_right_setpoint;
PID *motor_left_controller = NULL;
PID *motor_right_controller = NULL;

void motor_left_encoder_counter() {
  motor_left_encoder_position++;
}

void motor_right_encoder_counter() {
  motor_right_encoder_position++;
}

int motor_get_encoder_position(int m) {
  int position = 0;
  if (m & MOTOR_LEFT)
    position = motor_left_encoder_position;
  if (m & MOTOR_RIGHT)
    position = motor_right_encoder_position;
  return position;
}

void motor_reset_encoder(int m) {
  if (m & MOTOR_LEFT)
    motor_left_encoder_position = 0;
  if (m & MOTOR_RIGHT)
    motor_right_encoder_position = 0;
}

void motor_disable_encoder(int m) {
  if (m & MOTOR_LEFT)
    detachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ENC_PIN));
  if (m & MOTOR_RIGHT)
    detachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ENC_PIN));
  motor_reset_encoder(m);
}

void motor_enable_encoder(int m) {
  if (m & MOTOR_LEFT)
    attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ENC_PIN), motor_left_encoder_counter, FALLING);
  if (m & MOTOR_RIGHT)
    attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ENC_PIN), motor_right_encoder_counter, FALLING);
  motor_reset_encoder(m);
}

void motor_set_pid(int m, double kp, double ki, double kd) {
  if (m & MOTOR_LEFT) {
    motor_left_kp = motor_pid_data.motor_left_pid.kp = kp;
    motor_left_ki = motor_pid_data.motor_left_pid.ki = ki;
    motor_left_kd = motor_pid_data.motor_left_pid.kd = kd;
    motor_pid_data.status |= MOTOR_LEFT;
    EEPROM.put(0, motor_pid_data);
  }
  if (m & MOTOR_RIGHT) {
    motor_right_kp = motor_pid_data.motor_right_pid.kp = kp;
    motor_right_ki = motor_pid_data.motor_right_pid.ki = ki;
    motor_right_kd = motor_pid_data.motor_right_pid.kd = kd;
    motor_pid_data.status |= MOTOR_RIGHT;
    EEPROM.put(0, motor_pid_data);
  }
}

void motor_update(int m, int pwm) {
  byte dir = pwm > 0 ? HIGH : LOW;
  byte speed = MATH_ABS(pwm);
  if (m & MOTOR_LEFT) {
    digitalWrite(MOTOR_LEFT_ENB_PIN, dir);
    analogWrite(MOTOR_LEFT_PWM_PIN, speed);
  }
  if (m & MOTOR_RIGHT) {
    digitalWrite(MOTOR_RIGHT_ENB_PIN, dir);
    analogWrite(MOTOR_RIGHT_PWM_PIN, speed);
  }
  if (speed != 0)
    motor_enable_encoder(m);
  else
    motor_disable_encoder(m);
}

void motor_auto_turn(int m) {

  PIDAutotuner tuner = PIDAutotuner();

  // Set the target value to tune to
  // This will depend on what you are tuning. This should be set to a value within
  // the usual range of the setpoint. For low-inertia systems, values at the lower
  // end of this range usually give better results. For anything else, start with a
  // value at the middle of the range.
  tuner.setTargetInputValue(MOTOR_TUNNER_TARGET * MOTOR_PULSE_PER_ROUND);

  // Set the loop interval in microseconds
  // This must be the same as the interval the PID control loop will run at
  tuner.setLoopInterval(MOTOR_TUNNER_INTERVAL);

  // Set tuning cycles
  tuner.setTuningCycles(MOTOR_TUNE_CYCLES);

  // Set the output range
  // These are the maximum and minimum possible output values of whatever you are
  // using to control the system (analogWrite is 0-255)
  tuner.setOutputRange(MOTOR_PWM_MIN, MOTOR_PWM_MAX);

  // Set the Ziegler-Nichols tuning mode
  // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
  // or PIDAutotuner::ZNModeNoOvershoot. Test with ZNModeBasicPID first, but if there
  // is too much overshoot you can try the others.
  //  tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
  //  tuner.setZNMode(PIDAutotuner::ZNModeLessOvershoot);
  tuner.setZNMode(PIDAutotuner::ZNModeNoOvershoot);

  // This must be called immediately before the tuning loop
  tuner.startTuningLoop();

  // Run a loop until tuner.isFinished() returns true
  long milliseconds;
  while (!tuner.isFinished()) {

    // This loop must run at the same speed as the PID control loop being tuned
    long prevMilliseconds = milliseconds;
    milliseconds = millis();

    // Get input value here (temperature, encoder position, velocity, etc)
    int input = motor_get_encoder_position(m);
    motor_disable_encoder(m);

    // Call tunePID() with the input value
    double output = tuner.tunePID(input);

    // Set the output - tunePid() will return values within the range configured
    // by setOutputRange(). Don't change the value or the tuning results will be
    // incorrect.
    motor_update(m, output);

    // This loop must run at the same speed as the PID control loop being tuned
    while (millis() - milliseconds < MOTOR_TUNNER_INTERVAL) delay(1);
  }

  // Turn the output off here.
  motor_update(m, 0);

  // Get PID gains - set your PID controller's gains to these
  motor_set_pid(m, tuner.getKp(), tuner.getKi(), tuner.getKd());
}

void setup() {
  // Setup pin mode
  pinMode(MOTOR_LEFT_ENB_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_ENC_PIN, INPUT_PULLUP);

  pinMode(MOTOR_RIGHT_ENB_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_ENC_PIN, INPUT_PULLUP);


  // Setup serial
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {
    ;
  }

  // Read config from EPPROM
  memset(&motor_pid_data, 0, sizeof(motor_data));
  EEPROM.get(0, motor_pid_data);

  if (motor_pid_data.status > MOTOR_ALL) {
    motor_pid_data.status = 0;
  }

  // Get pid config from epprom or run auto turn
  if (motor_pid_data.status & MOTOR_LEFT) {
    motor_left_kp = motor_pid_data.motor_left_pid.kp;
    motor_left_ki = motor_pid_data.motor_left_pid.ki;
    motor_left_kd = motor_pid_data.motor_left_pid.kd;
  } else {
    motor_auto_turn(MOTOR_LEFT);
  }

  if (motor_pid_data.status & MOTOR_RIGHT) {
    motor_right_kp = motor_pid_data.motor_right_pid.kp;
    motor_right_ki = motor_pid_data.motor_right_pid.ki;
    motor_right_kd = motor_pid_data.motor_right_pid.kd;
  } else {
    motor_auto_turn(MOTOR_RIGHT);
  }

  // Stop all motor
  motor_update(MOTOR_ALL, 0);
  delay(2000);

  // Setup PID controller
  motor_left_output = MOTOR_PWM_MIN;
  motor_left_controller = new PID(&motor_left_input, &motor_left_output, &motor_left_setpoint, motor_left_kp, motor_left_ki, motor_left_kd, DIRECT);
  motor_left_controller->SetMode(AUTOMATIC);
  motor_left_controller->SetSampleTime(MOTOR_TUNNER_INTERVAL);
  motor_left_controller->SetOutputLimits(-1 * MOTOR_PWM_MAX, MOTOR_PWM_MAX);

  motor_right_output = MOTOR_PWM_MIN;
  motor_right_controller = new PID(&motor_right_input, &motor_right_output, &motor_right_setpoint, motor_right_kp, motor_right_ki, motor_right_kd, DIRECT);
  motor_right_controller->SetMode(AUTOMATIC);
  motor_right_controller->SetSampleTime(MOTOR_TUNNER_INTERVAL);
  motor_right_controller->SetOutputLimits(-1 * MOTOR_PWM_MAX, MOTOR_PWM_MAX);
}

void loop() {
  while (1) {
    long milliseconds =  millis();

    motor_left_setpoint = MOTOR_TUNNER_TARGET * MOTOR_PULSE_PER_ROUND;
    motor_left_input = motor_get_encoder_position(MOTOR_LEFT);
    motor_disable_encoder(MOTOR_LEFT);
    motor_left_controller->Compute();
    motor_left_output = MATH_ABS(motor_left_output);
    motor_update(MOTOR_LEFT, motor_left_output);

    Serial.print("Setpoint: ");
    Serial.print(motor_left_setpoint);
    Serial.print(" Input: ");
    Serial.print(motor_left_output);
    Serial.println();

    motor_right_setpoint = MOTOR_TUNNER_TARGET * MOTOR_PULSE_PER_ROUND;
    motor_right_input = motor_get_encoder_position(MOTOR_RIGHT);
    motor_disable_encoder(MOTOR_RIGHT);
    motor_right_controller->Compute();
    motor_right_output = MATH_ABS(motor_right_output);
    motor_update(MOTOR_RIGHT, motor_right_output);

    Serial.print("Setpoint: ");
    Serial.print(motor_right_setpoint);
    Serial.print(" Input: ");
    Serial.print(motor_right_input);
    Serial.println();

    while (millis() - milliseconds < MOTOR_TUNNER_INTERVAL) delay(1);
  }
}
