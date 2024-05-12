#include <AccelStepper.h>
#include <PID_v1.h>

// EL1952G5
// f*EmqmQ36Tb9jWB

// number of axes
//******IMPORTANT*********
const int stepper_servo_axes = 4;

// these global angle values manage program's target angle
float actual_angles[stepper_servo_axes];    //current measured angles from encoders
float target_angles[stepper_servo_axes] = {0};    //angles each axis tries to move to 
float old_target_angles[stepper_servo_axes] = {0};    //p

bool new_target_angles[stepper_servo_axes] = {false, false, false, false};

// these are global status variables and acceptable values
String global_status = "";
bool new_global_status = false;
const String e_stop_status_msg = "estop";
const String joint_hold_status_msg = "joint_hold";
const String end_effector_open_status_msg = "end_effector:open";
const String end_effector_close_status_msg = "end_effector:close";

// internvals between when functions should run in microsecs
const int serial_update_runtime_microsecs = 500;
const int end_effector_runtime_microsecs = 1500;
const int e_stop_runtime_microsteps = 250;
const int joint_hold_runtime_microsteps = 250;
const int runtime_interval_microsecs[stepper_servo_axes] = {750, cur, 350, 350};

// global time var for syncing

long global_time;

// joint param
const int motor_pins[stepper_servo_axes][2] = {
    {23, 25},
    {27, 29},
    {31, 33},
    {35, 37}}; // pins motors are plugged into, order {step, direction}
const int microsteps_reduction_params[stepper_servo_axes][2] = {
    {16, 20},
    {16, -15},
    {8, 15},
    {8, 3}};                                                                                                                                  // gear stepper microsteps and gear reduction for each axes, order {microstep, reduction}
const long speed_accel_params[stepper_servo_axes][2] = {{2 * 2000, 2 * 2000}, {80000, 4 * 5000}, {3 * 50000, 75000}, {2 * 15000, 2 * 15000}}; // speed and acceleration params of motor, in steps/second (including microsteps), order: {speed, acceleration}
const int joint_lims[stepper_servo_axes][2] = {
    {-180, 180},
    {-20, 20},
    {-180, 180},
    {-180, 180}}; // joint limits for each axis, the only ones that are really updated are for axis 2 as of 5/3
const long PID_params[stepper_servo_axes][3] = {
    {1, 1, 1},
    {.1, .1, .1},
    {1, 1, 1},
    {1, 1, 1}};                                                                                      // pid params
const float angle_offset_sign[stepper_servo_axes][2] = {{0, 1}, {-261 + 78.29, -1}, {0, 1}, {0, 1}}; // angle offset for encoder input, first value is added and the second is multiplied by the encoder input

// these two functions are used internally for convenience
inline bool check_runtime(long inp_last_time, int inp_runtime_interval_microsecs)
{
  // checks if time since last runtime is enough for program to continue
  if (global_time - inp_last_time > inp_runtime_interval_microsecs)
  {
    return true;
  }
  return false;
}

class StepperServo
{
public:
  bool is_running = false;
  StepperServo(int inp_joint_num)
  {

    joint_num = inp_joint_num;
    stepper_motor = new AccelStepper(AccelStepper::DRIVER, motor_pins[joint_num][0], motor_pins[joint_num][1]);
    stepper_motor->setMaxSpeed(speed_accel_params[joint_num][0]);
    stepper_motor->setAcceleration(speed_accel_params[joint_num][1]);

    PID_controller = new PID((long)(&actual_angles[joint_num]), (long)(&output_val), (long)(&target_angles[joint_num]), PID_params[joint_num][0], PID_params[joint_num][1], PID_params[joint_num][2], DIRECT);
    PID_controller->SetMode(AUTOMATIC);

    check_current_angle();

    Serial.print("joint ");
    Serial.print(joint_num);
    Serial.print(" init success (index starts at 0), current joint angle: ");
    Serial.print(actual_angles[joint_num]);
    Serial.print(" degrees. Moving to ");
    Serial.print(target_angles[joint_num]);
    Serial.println(" degrees");
  }

  void main()
  {
    if (check_runtime(last_time, runtime_interval_microsecs[joint_num]))
    {
      check_current_angle();

      PID_controller->Compute();
      stepper_motor->move(output_val * (200.0 / 360.0) * microsteps_reduction_params[joint_num][0] * microsteps_reduction_params[joint_num][1]);
      // Serial.print(actual_angles[joint_num]);
      Serial.println(" ");
      // Serial.println(output_val * (200.0 / 360.0) * microsteps_reduction_params[joint_num][0] * microsteps_reduction_params[joint_num][1]);
      // Serial.println(output_val);
      // float meow = (target_angles[joint_num] - actual_angles[joint_num]) * 360 * microsteps_reduction_params[joint_num][0] * microsteps_reduction_params[joint_num][1];
      // float meow = (last_angle - target_angles[joint_num]) * 200 * microsteps_reduction_params[joint_num][0] * microsteps_reduction_params[joint_num][1];

      // meow = meow / 360;
      // stepper_motor->move(meow);

      /*
      while (stepper_motor->distanceToGo() != 0) {
        stepper_motor->run();
        //Serial.println("blocking");
      }*/

      if (stepper_motor->distanceToGo() != 0)
      {
        do
        {
          stepper_motor->run();
          yield();
        } while (stepper_motor->distanceToGo() != 0 && !is_running);
      }
      else
      {
      }
    }
  }

private:
  AccelStepper *stepper_motor;
  PID *PID_controller;

  int joint_num;

  long last_time = 0;
  float last_angle = 0;
  float output_val;

  void check_current_angle()
  {
    float angle = (360 * (float)(analogRead(joint_num)) / 664);
    angle = angle_offset_sign[joint_num][0] + angle * angle_offset_sign[joint_num][1] - 90;
    if (angle > 180)
    {
      angle -= 360;
    }
    else if (angle < -180)
    {
      angle += 360;
    }

    actual_angles[joint_num] = angle;
  }
};

const int debug_axis = 1;
StepperServo *test_StepperServo;

void setup()
{
  Serial.begin(9600);
  Serial.println("begin setup!");
  test_StepperServo = new StepperServo(debug_axis);
  Serial.println("end setup!");
}

bool up = true;
bool down = false;

long last = millis();
long last_print = 0;
int interval = 20000;
bool start = true;

void loop()
{
  global_time = micros();
  test_StepperServo->main();

  // Serial.println(actual_angles[debug_axis]);
  /*

  if ((up && (millis() - last > interval) && !down) || start) {
    start = false;
    Serial.println("up");

    target_angles[debug_axis] = 5;
    up = false;
    down = true;
    last = millis();
  }

  if (down && (millis() - last > interval) && !up) {
    Serial.println("down");
    target_angles[debug_axis] = -5;
    up = true;
    down = false;
    last = millis();
  }*/
  target_angles[debug_axis] = -10;
}
