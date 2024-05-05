#include <AccelStepper.h>
#include <PID_v1.h>

// EL1952G5
// f*EmqmQ36Tb9jWB

// number of axes
//******IMPORTANT*********
const int stepper_servo_axes = 4;

// these global angle values manage program's target angle
float actual_angles[stepper_servo_axes];
float target_angles[stepper_servo_axes] = {0};
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
const int runtime_interval_microsecs[stepper_servo_axes] = {750, 750, 350, 350};

// global time var for syncing

long global_time;

// joint param
const int motor_pins[stepper_servo_axes][2] = {
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}}; // pins motors are plugged into, order {step, direction}
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
    {1, 1, 1},
    {1, 1, 1},
    {1, 1, 1}};                                                                                // pid params
const float angle_offset_sign[stepper_servo_axes][2] = {{0, 1}, {185.15, -1}, {0, 1}, {0, 1}}; // angle offset for encoder input, first value is added and the second is multiplied by the encoder input

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

bool check_if_current_status(String condition) // checks if status is condition, thne does appropriate actions if true
{
  if (condition == global_status && new_global_status == true)
  {
    global_status = "";
    new_global_status = false;
    return true;
  }
  return false;
}

// next three functions parse serial input and put it in appropriate var
void update_from_serial()
{
  static long last_time = 0;
  static bool first_char = true;
  static bool status_update;

  const char angle_init_char = 'A';
  const char status_init_char = 'S';
  const char end_line_char = '\n';

  static String parsed_string = "";

  if (check_runtime(last_time, serial_update_runtime_microsecs))
  {
    last_time = global_time;
    if (Serial.available())
    {
      char read_char = Serial.read();
      if (first_char && read_char == status_init_char)
      { // check if first char of status update msg
        status_update = true;
      }
      else if (first_char && read_char == angle_init_char)
      { // check if first char of angle update msg
        status_update = false;
      }
      else if (read_char == end_line_char)
      {                                                                                     // check if end msg char
        status_update ? update_status(parsed_string) : update_target_angles(parsed_string); // call appropriate string handling function
        parsed_string = "";                                                                 // clear input string
      }
      else
      { // if not others, then its char partway through string so append to inp string
        parsed_string += read_char;
      }
    }
  }
}

void update_target_angles(String input_string)
{

  const char deliminator_char = ',';
  String parsed_angle = "";

  uint8_t angle_counter_coord = 0;

  for (char angle_char : input_string)
  {
    if (angle_char != deliminator_char)
    {
      parsed_angle += angle_char;
    }
    else
    {
      target_angles[angle_counter_coord] = atof(parsed_angle.c_str());
      parsed_angle = "";
      angle_counter_coord++;
    }
  }

  target_angles[angle_counter_coord] = atof(parsed_angle.c_str());
  parsed_angle = "";

  for (int i = 0; i < stepper_servo_axes; i++)
  {
    new_target_angles[i] = true;
  }
}

void update_status(String input_string)
{

  const String acceptable_statuses[4] = {e_stop_status_msg, joint_hold_status_msg, end_effector_open_status_msg, end_effector_close_status_msg};

  for (String status : acceptable_statuses)
  {
    if (status == input_string)
    {
      global_status = status;
      new_global_status = true;
      return;
    }
  }
  Serial.print("last status: '");
  Serial.print(input_string);
  Serial.println("' not recognized, disregarding");
}

// these functions manage safety stuff
void manage_joint_hold()
{

  static long last_time = 0;
  static bool joint_hold_on = false;

  const int joint_hold_button = 14;
  const int joint_hold_threshold = 500;

  if (check_runtime(last_time, joint_hold_runtime_microsteps))
  {
    last_time = global_time;
    if ((analogRead(joint_hold_button) > joint_hold_threshold) || check_if_current_status(joint_hold_status_msg))
    {
      if (joint_hold_on)
      {
        joint_hold_on = false;
        joint_unhold();
      }
      else
      {
        joint_hold_on = true;
        joint_hold();
      }
    }
  }
}

void joint_hold()
{
  for (int i = 0; i < stepper_servo_axes; i++) // this function actually stops the arm, sets the target angles to current, so arm doesn't want to move
  {
    StepperServo_array[i]->is_running = false;
    target_angles[i] = actual_angles[i];
  }
  Serial.print("joint hold on, angles: ");
  for (int i = 0; i < stepper_servo_axes; i++)
  {
    Serial.print(target_angles[i]);
    Serial.print(" degrees");
  }
}

void joint_unhold()
{
  for (int i = 0; i < stepper_servo_axes; i++)
  {
    StepperServo_array[i]->is_running = true;
  }
  Serial.println("joint hold off");
}

void manage_e_stop()
{
  static long last_time = 0;
  const int e_stop_button = 15;
  const int e_stop_threshold = 500;

  if (check_runtime(last_time, e_stop_runtime_microsteps))
  {
    last_time = global_time;
    if ((analogRead(e_stop_button) > e_stop_threshold) || check_if_current_status(e_stop_status_msg))
    {
      // this functions basically the same as above joint hold func but it also sends the program into a permenant loop, stopping it
      // when able i should update electronics to allow for digital control of stepper dribver enablepins for an actual power-off e stop

      for (int i = 0; i < stepper_servo_axes; i++)
      {
        target_angles[i] = actual_angles[i];
        StepperServo_array[i]->is_running = false;
      }
      Serial.println("/////////////// E STOP ACTIVATED ///////////////");
      Serial.print("arm joint stopped, angles: ");
      for (int i = 0; i < stepper_servo_axes; i++)
      {
        Serial.print(target_angles[i]);
        Serial.println(" degrees");
        Serial.println("restart microcontroller to continue ");
      }
      while (true) // permantenly halts program in case of e stop event
      {
        delay(10);
      }
    }
  }
}

// class that combines steppers and encoders into unified servo object
class StepperServo
{
public:
  bool is_running = true;

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
      stepper_motor->move(output_val * (200 / 360) * microsteps_reduction_params[joint_num][0] * microsteps_reduction_params[joint_num][1]);

      if (stepper_motor->distanceToGo() != 0)
      {
        do
        {
          stepper_motor->run();
          yield();
        } while (stepper_motor->distanceToGo() != 0 && is_running);
      }
    }
  }

private:
  AccelStepper *stepper_motor;
  PID *PID_controller;

  int joint_num;
  long last_time = 0;
  float output_val;

  void check_current_angle()
  {
    float angle = (360 * (float)(analogRead(joint_num)) / 664);
    angle = angle_offset_sign[joint_num][0] + angle * angle_offset_sign[joint_num][1];
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

// create joint array
StepperServo *StepperServo_array[stepper_servo_axes];

void setup()
{
  Serial.begin(9600);
  Serial.println("begin setup!");

  for (int i = 0; i < stepper_servo_axes; i++)
  {
    StepperServo_array[i] = new StepperServo(i); // remember args
  }

  Serial.println("");
  Serial.println("end setup!");
}

void loop()
{
  global_time = micros();

  for (int i = 0; i < stepper_servo_axes; i++)
  {
    StepperServo_array[i]->main();
  }
  update_from_serial();
  manage_e_stop();
  manage_joint_hold();
}
