long global_time;            // global master time variable
int stepper_servo_axes = 4;  // number of xtepper servo joints **IMPORTANT**

float target_angles[4] = { 0, 0, 0, 0 };  // arm target angles
bool new_target_angles = false;           // turns true if there are new target angles

String global_status = "";                                                                                       // global master status variable
bool new_global_status = false;                                                                                  // turns true if there is a new status
const String allowed_global_statuses[4] = { "estop", "joint_hold", "end_effector:open", "end_effector:close" };  // list of acceptable status

const int serial_update_runtime_microsecs = 100;  // how long the program should run for

inline bool check_runtime(long inp_last_time, int inp_runtime_interval_microsecs) {  // returns true if enough time has passed since last runtime
  bool should_run = (global_time - inp_last_time > inp_runtime_interval_microsecs) ? true : false;
  return should_run;
}

void update_from_serial() {  // takes data from serial, checks if it is an angle update or status update, then passes it to appropriate function
  static long last_time = 0;
  static bool first_char = true;
  static bool status_update;

  const char angle_init_char = 'A';
  const char status_init_char = 'S';
  const char end_line_char = '\n';

  static String parsed_string = "";

  if (check_runtime(last_time, serial_update_runtime_microsecs)) {
    last_time = global_time;
    if (Serial.available()) {
      char read_char = Serial.read();
      if (first_char && read_char == status_init_char) {  // check if first char of status update msg
        status_update = true;
      } else if (first_char && read_char == angle_init_char) {  // check if first char of angle update msg
        status_update = false;
      } else if (read_char == end_line_char) {                                               // check if end msg char
        status_update ? update_status(parsed_string) : update_target_angles(parsed_string);  // call appropriate string handling function
        parsed_string = "";                                                                  // clear input string
      } else {                                                                               // if not others, then its char partway through string so append to inp string
        parsed_string += read_char;
      }
    }
  }
}

void update_target_angles(String input_string) {  // angle updating function, turns inp string ->float[]

  const char deliminator_char = ',';
  String parsed_angle = "";

  uint8_t angle_counter_coord = 0;

  for (char angle_char : input_string) {
    if (angle_char != deliminator_char) {
      parsed_angle += angle_char;
    } else {
      target_angles[angle_counter_coord] = atof(parsed_angle.c_str());
      parsed_angle = "";
      angle_counter_coord++;
    }
  }

  target_angles[angle_counter_coord] = atof(parsed_angle.c_str());
  parsed_angle = "";

  new_target_angles = true;
}

void update_status(String input_string) {  // status updating function, varifies validity of status and outputs it

  global_status = input_string;
  new_global_status = true;
}

void setup() {
  pinMode(9, OUTPUT);
  Serial.begin(9600);
  Serial.println("setup");
}

void loop() {
  global_time = micros();
  update_from_serial();
  if (new_target_angles) {
    while (true) {
      bool meow = true;
      int test[4] = { 12, 56, 0, 12 };
      for (int i = 0; i < 4; i++) {
        if (target_angles[i] != test[i]) { meow = false; }
      }
      while (meow) {
        digitalWrite(9, HIGH);
        delay(100);
        digitalWrite(9, LOW);
        delay(100);
      }
      new_target_angles = false;
    }
  }
}