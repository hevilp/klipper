// Code for controlling a "servo stepper"
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_disable
#include "board/gpio.h" // gpio_pwm_write
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "driver_a4954.h" // a4954_oid_lookup
#include "servo_stepper.h" // servo_stepper_oid_lookup
#include "virtual_stepper.h" // virtual_stepper_oid_lookup
#include "sched.h" // shutdown

#define CONSTRAIN(val,min,max)(         \
    ((val) < (min)) ? (min) :           \
    (((val) > (max)) ? (max) : (val)))
#define ABS(val) (((val) < 0) ? -(val) : (val))

#define PID_INIT_SAMPLES 9
#define PID_SCALE_DIVISOR 1024
// TODO:  TIME_SCALE_SHIFT needs to be calculated based on the clock frequency
// To get the expected number of nano seconds it takes to run the loop at 6KHz:
// ns_per_update = CONFIG_CLOCK_FREQ / 166667
// Divide by 60 for scale:
// divisor = ns_per_update / 10
// Then we need to get the location of the MSB set to 1
// Not sure how to get the preprocessor to do this.
#define TIME_SCALE_SHIFT 20
#define PID_ALLOWABLE_ERROR 16
#define FULL_STEP 256

struct pid_control {
    uint8_t init_count;
    int16_t Kp, Ki, Kd;
    int32_t integral;
    int32_t error;
    uint32_t last_enc_pos;
    uint32_t last_stp_pos;
    uint32_t last_sample_time;
};

struct servo_stepper {
    struct a4954 *stepper_driver;
    struct virtual_stepper *virtual_stepper;
    struct pid_control pid_ctrl;
    uint32_t full_steps_per_rotation;
    uint32_t excite_angle;
    uint32_t run_current_scale, hold_current_scale;
    uint8_t flags;
};

enum {
    SS_MODE_DISABLED = 0, SS_MODE_OPEN_LOOP = 1, SS_MODE_TORQUE = 2,
    SS_MODE_HPID = 3, SS_MODE_PID_INIT = 4
};

static uint32_t
position_to_phase(struct servo_stepper *ss, uint32_t position)
{
    return DIV_ROUND_CLOSEST(
        ss->full_steps_per_rotation * position, FULL_STEP);
}

static void
servo_stepper_mode_open_loop(struct servo_stepper *ss, uint32_t position)
{
    uint32_t vs_position = virtual_stepper_get_position(ss->virtual_stepper);
    a4954_set_phase(ss->stepper_driver, vs_position, ss->run_current_scale);
}

static void
servo_stepper_mode_torque_update(struct servo_stepper *ss, uint32_t position)
{
    uint32_t phase = position_to_phase(ss, position);
    a4954_set_phase(ss->stepper_driver, phase + ss->excite_angle
                    , ss->run_current_scale);
}

static void
servo_stepper_mode_pid_init(struct servo_stepper *ss, uint32_t position)
{
    if (ss->pid_ctrl.init_count == 0) {
        // Set the initial reference position
        ss->pid_ctrl.last_enc_pos = position;
    } else {
        int32_t pos_diff = position - ss->pid_ctrl.last_enc_pos;
        if (ABS(pos_diff) > FULL_STEP)
            shutdown("Encoder Variance too large!  Check your calibration"
                " and magnet position.");
        // Temporarily store the difference in the error member,
        // since it is a signed integer
        ss->pid_ctrl.error += pos_diff;
    }

    ss->pid_ctrl.init_count++;

    if (ss->pid_ctrl.init_count >= PID_INIT_SAMPLES) {
        int32_t deviation = DIV_ROUND_CLOSEST(
            ss->pid_ctrl.error, (PID_INIT_SAMPLES - 1));
        uint32_t avg_pos = ss->pid_ctrl.last_enc_pos + deviation;
        output("Encoder deviation: %i, average pos: %u",
            deviation, avg_pos);
        ss->pid_ctrl.error = 0;
        ss->pid_ctrl.last_enc_pos= avg_pos;
        ss->pid_ctrl.last_stp_pos = virtual_stepper_get_position(
            ss->virtual_stepper);
        ss->pid_ctrl.last_sample_time = timer_read_time();
        ss->flags = SS_MODE_HPID;
    }
}

static void
servo_stepper_mode_hpid_update(struct servo_stepper *ss, uint32_t position)
{
    // hpid = hybrid pid
    // The idea behind hybrid PID is to skip the PID loop if the stepper was
    // not commanded to move AND the measured position is within an acceptable
    // error. In the initial test I will allow a phase of +/- 16 microsteps,
    // as this seems like reasonable accuracy for the encoder to maintain.

    // TODO: Implement Alpha-Beta Filter on the Encoder Position using
    // the stepper phase current

    uint32_t sample_time = timer_read_time();
    uint32_t time_diff = ((sample_time - ss->pid_ctrl.last_sample_time)
        >> TIME_SCALE_SHIFT) + 1;
    uint32_t stp_pos = virtual_stepper_get_position(ss->virtual_stepper);
    // TODO:  Check the math below to make sure the cast works correctly
    int32_t measured_diff = (int32_t)((position - ss->pid_ctrl.last_enc_pos)
        * ss->full_steps_per_rotation) / FULL_STEP;
    int32_t move_diff = stp_pos - ss->pid_ctrl.last_stp_pos;
    ss->pid_ctrl.error += move_diff - measured_diff;

    // Calculate the i-term;
    ss->pid_ctrl.integral += ss->pid_ctrl.error * time_diff;
    ss->pid_ctrl.integral = CONSTRAIN(
        ss->pid_ctrl.integral, -FULL_STEP, FULL_STEP);

    if ((ABS(ss->pid_ctrl.error) <= PID_ALLOWABLE_ERROR) && !move_diff) {
        // Error is within the allowable threshold and no additional movement
        // has been requested, so we can hold
        a4954_set_phase(ss->stepper_driver, stp_pos, ss->hold_current_scale);
    } else {
        // Enter the PID Loop
        int32_t co = ((ss->pid_ctrl.Kp * ss->pid_ctrl.error) +
            (ss->pid_ctrl.Ki * ss->pid_ctrl.integral) -
            (ss->pid_ctrl.Kd * measured_diff / time_diff)) / PID_SCALE_DIVISOR;
        co = CONSTRAIN(co, -FULL_STEP, FULL_STEP);
        uint32_t cur_scale = ((ABS(co) * (ss->run_current_scale -
            ss->hold_current_scale)) >> 8) + ss->hold_current_scale;
        uint32_t phase = stp_pos - ss->pid_ctrl.error + co;
        a4954_set_phase(ss->stepper_driver, phase, cur_scale);
    }

    ss->pid_ctrl.last_enc_pos = position;
    ss->pid_ctrl.last_stp_pos = stp_pos;
    ss->pid_ctrl.last_sample_time = sample_time;
}

void
servo_stepper_update(struct servo_stepper *ss, uint32_t position)
{
    uint32_t mode = ss->flags;
    switch (mode) {
    case SS_MODE_OPEN_LOOP: servo_stepper_mode_open_loop(ss, position); break;
    case SS_MODE_TORQUE: servo_stepper_mode_torque_update(ss, position); break;
    case SS_MODE_HPID: servo_stepper_mode_hpid_update(ss, position); break;
    case SS_MODE_PID_INIT: servo_stepper_mode_pid_init(ss, position); break;
    }
}

void
command_config_servo_stepper(uint32_t *args)
{
    struct a4954 *a = a4954_oid_lookup(args[1]);
    struct virtual_stepper *vs = virtual_stepper_oid_lookup(args[2]);
    struct servo_stepper *ss = oid_alloc(
        args[0], command_config_servo_stepper, sizeof(*ss));
    ss->stepper_driver = a;
    ss->virtual_stepper = vs;
    ss->full_steps_per_rotation = args[3];
}
DECL_COMMAND(command_config_servo_stepper,
             "config_servo_stepper oid=%c driver_oid=%c stepper_oid=%c"
             " full_steps_per_rotation=%u");

struct servo_stepper *
servo_stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_servo_stepper);
}

static void
servo_stepper_set_disabled(struct servo_stepper *ss)
{
    irq_disable();
    ss->flags = SS_MODE_DISABLED;
    a4954_disable(ss->stepper_driver);
    irq_enable();
}

static void
servo_stepper_set_open_loop_mode(struct servo_stepper *ss, uint32_t *args)
{
    irq_disable();
    a4954_enable(ss->stepper_driver);
    ss->flags = SS_MODE_OPEN_LOOP;
    ss->run_current_scale = args[2];
    ss->hold_current_scale = args[3];
    irq_enable();
}

static void
servo_stepper_set_hpid_mode(struct servo_stepper *ss, uint32_t *args)
{
    irq_disable();
    a4954_enable(ss->stepper_driver);
    ss->run_current_scale = args[2];
    ss->hold_current_scale = args[3];
    ss->pid_ctrl.Kp = args[4];
    ss->pid_ctrl.Ki = args[5];
    ss->pid_ctrl.Kd = args[6];
    ss->pid_ctrl.init_count = 0;
    ss->pid_ctrl.last_enc_pos = 0;
    ss->pid_ctrl.last_stp_pos = 0;
    ss->pid_ctrl.error = 0;
    ss->pid_ctrl.integral = 0;
    ss->flags = SS_MODE_PID_INIT;
    irq_enable();
}

static void
servo_stepper_set_torque_mode(struct servo_stepper *ss, uint32_t *args)
{
    irq_disable();
    a4954_enable(ss->stepper_driver);
    ss->flags = SS_MODE_TORQUE;
    ss->run_current_scale = args[2];
    ss->excite_angle = args[3];
    irq_enable();
}

void
command_servo_stepper_set_mode(uint32_t *args)
{
    // Note:  The flex arg (arg[3]) can be the hold_current_scale or
    // excite_angle
    struct servo_stepper *ss = servo_stepper_oid_lookup(args[0]);
    uint8_t mode = args[1];
    switch(mode) {
        case 0:
            servo_stepper_set_disabled(ss);
            break;
        case 1:
            servo_stepper_set_open_loop_mode(ss, args);
            break;
        case 2:
            servo_stepper_set_torque_mode(ss, args);
            break;
        case 3:
            servo_stepper_set_hpid_mode(ss, args);
            break;
        default:
            shutdown("Unknown Servo Mode");
    }
}
DECL_COMMAND(command_servo_stepper_set_mode,
             "servo_stepper_set_mode oid=%c mode=%c run_current_scale=%u"
             " flex=%u kp=%hi ki=%hi kd=%hi");

void
command_servo_stepper_get_stats(uint32_t *args)
{
    uint8_t oid = args[0];
    struct servo_stepper *ss = servo_stepper_oid_lookup(oid);
    sendf("servo_stepper_stats oid=%c error=%i", oid, ss->pid_ctrl.error);

}
DECL_COMMAND(command_servo_stepper_get_stats,
             "servo_stepper_get_stats oid=%c");
