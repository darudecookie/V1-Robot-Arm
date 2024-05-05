#include <AccelStepper.h>
#include <PID_v1.h>

class StepperServo
{
public:
    /*
      StepperServo()
      {
          Serial.println("init fail - no constructor args");
      }
      */

    StepperServo(int inp_joint_num, int inp_microsteps_reduction_params[2], long inp_speed_accel_params[2], int inp_joint_lims[2], long inp_PID_params[3], float inp_angle_offset, int inp_runtime_interval_microsecs) //    : inp_joint_num(joint_num), inp_microsteps_reduction_params(microsteps_reduction_params), inp_speed_accel_params(speed_accel_params), inp_joint_lims(joint_lims), inp_PID_params(PID_params), inp_angle_offset(angle_offset), inp_runtime_interval_microsecs(runtime_interval_microsecs) {
    {

        // joint_num = inp_joint_num;
        // runtime_interval_microsecs = inp_runtime_interval_microsecs;
        // angle_offset = inp_angle_offset;

        /*
            for (int i = 0; i < 2; i++) {
              microsteps_reduction_params[i] = inp_microsteps_reduction_params[i];
              speed_accel_params[i] = inp_speed_accel_params[i];
              joint_lims[i] = inp_joint_lims[i];
              PID_params[i] = inp_PID_params[i];
            }
            PID_params[2] = inp_PID_params[2];
        */
        static AccelStepper stepper_motor;
        stepper_motor.setMaxSpeed(speed_accel_params[0]);
        stepper_motor.setAcceleration(speed_accel_params[1]);

        // current_angle = check_current_angle();
        // target_angle = 0;

        // static PID PID_controller(&current_angle, &output_val, &target_angle, PID_params[0], PID_params[1], PID_params[2], DIRECT);
        // PID_controller.SetMode(AUTOMATIC);

        Serial.print("joint ");
        // Serial.print(joint_num);
        Serial.println(" init success (index starts at 0)");
    }

    float check_current_angle()
    {
        // float angle = analogRead(joint_num) * (5 / 1023);
        float angle = 0;
        // angle += angle_offset;
        if (angle > 180)
        {
            angle -= 360;
        }
        else if (angle < -180)
        {
            angle += 360;
        }

        return angle;
    }

    void main(long global_time)
    {
        Serial.println("hello world");
    }

private:
    long last_time = 0;

    int joint_num, runtime_interval_microsecs;
    int microsteps_reduction_params[2];
    int joint_lims[2];
    long PID_params[3];
    long speed_accel_params[2];
    float angle_offset;

    double current_angle, target_angle, output_val;

    inline bool check_runtime(long inp_global_time)
    {
        bool should_run = (inp_global_time - StepperServo::last_time > StepperServo::runtime_interval_microsecs) ? true : false;
        return true;
    }
};
