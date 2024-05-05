#include <AccelStepper.h>
#include <PID_v1.h>

// EL1952G5
// f*EmqmQ36Tb9jWB

const int stepper_servo_axes = 4;

// global control/communication variables
float actual_angles[stepper_servo_axes];

float target_angles[stepper_servo_axes] = {0};
bool new_target_angles = false;

String global_status = "";
bool new_global_status = false;
const String allowed_global_statuses[4] = {"estop", "joint_hold", "end_effector:open", "end_effector:close"};

const int serial_update_runtime_microsecs = 100;
const int end_effector_runtime_microsecs = 500;
const int runtime_interval_microsecs[stepper_servo_axes] = {750, 750, 350, 350};

// global time var for syncing
long global_time;

uint8_t end_effector_status()
{
    static long last_time = 0;
    const String end_effector_open = allowed_global_statuses[2];
    const String end_effector_close = allowed_global_statuses[3];

    if (check_runtime(last_time, end_effector_runtime_microsecs) && new_global_status)
    {
        last_time = micros();
        if (new_global_status == end_effector_open)
        {
            new_global_status = false;
        }
        else if (new_global_status == end_effector_close)
        {
            new_global_status = false;
        }
        else
    }
}

inline bool check_runtime(long inp_last_time, int inp_runtime_interval_microsecs)
{
    if (global_time - inp_last_time > inp_runtime_interval_microsecs)
    {
        return true;
    }
    else
    {
        return false;
    }
}
