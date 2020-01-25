// Code for controlling a "servo stepper"
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_disable
#include "board/gpio.h" // gpio_pwm_write
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "driver_a4954.h" // a4954_oid_lookup
#include "servo_stepper.h" // servo_stepper_oid_lookup
#include "virtual_stepper.h" // virtual_stepper_oid_lookup
#include "sched.h" // shutdown

#define CONSTRAIN(val,min,max)(     \
    (val) < (min) ? (min) :        \
    ((val) > (max) ? (max) : (val)))
#define ABS(val)((val) < 0 ? -(val) : (val))
#define PID_SCALE_SHIFT 10
// TODO:  We probably should measure the real update time rather than assume
// we are getting it every 160us
#define PID_ALLOWABLE_ERROR 32
#define POS_UPDATE_TIME 160

struct pid_control {
    int16_t Kp, Ki, Kd;
    int32_t last_error;
    uint32_t last_position, last_sample_time;
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
    return DIV_ROUND_CLOSEST(ss->full_steps_per_rotation * position, 256);
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
servo_stepper_mode_hpid_update(struct servo_stepper *ss, uint32_t position)
{
    // hpid = hybrid pid
    // The idea behind hybrid PID is to skip the PID loop if the stepper was not
    // commanded to move AND the measured position is within an acceptable error.
    // In the initial test I will allow a phase
    // of +/- 32 (8 microsteps), as this seems like a reasonable
    //
    // TODO:  I may need to check to see if the incoming position's MSB is 1,
    // as that would indicate that the encoder is traveled past 0 in the reverse
    // direction.  If the direction the stepper travels is inverted from the direction
    // the encoder measures we need to find a way to correct for that as well.
    //
    // position_offset needs to be calculated in the init sequence.  Essentially
    // it needs to be the difference between the encoder and the stepper position,
    // the questions do I need it?  During init I could sample the encoder position,
    // average the value, convert it to phase, then set that as the virtual stepper position.
    // I think that is what Kevin intended.  Its probably better to query the position and
    // do the average on the host.
    //
    // We will also likely need to store the previous measured phase to do the deriviative
    // correctly
    //
    uint32_t current_phase = position_to_phase(ss, position);
    uint32_t desired_pos = virtual_stepper_get_position(ss->virtual_stepper);

    // TODO: need to handle wrap around/overflow to correctly get the error
    int32_t error = desired_pos - current_phase;
    if (ABS(error) < PID_ALLOWABLE_ERROR) {
        return;
    }

    // PID algoritm
    // error = desired position (stepper postion) - read position (phase)
    // P_term = kp * error
    // I_term = prevoius_I_term + (ki * current_update_time - prev_update_time)
    // **** The I term should be bound to +/- one full step (+/- 256 I beleive)
    // D_term = kd * ((current_measured_phase - previous_measured_phase) / (current_update_time - prev_update_time))
    // **** Note that its possible to create a "min derivative time" variable to change how the D_term is calculated and
    // thus how it behaves.  Increasing should re
    //
    // CO = P_term + I_term - D_term
    // *** CO should be bound to one full step.  Whe calculating current, it should
    // be scaled to the correct range and bound within hold and run currents
    //
    // Each of kp, ki, and kd are scaled, as they are input as floating point values
    // I need to see the best way to do this.
    //
    // Since time is returned in clock ticks, I also need to figure out the best way to represent it
    // without making all of my values zero.  I really need to just use the monotonic clock

}

void
servo_stepper_update(struct servo_stepper *ss, uint32_t position)
{
    uint32_t mode = ss->flags;
    switch (mode) {
    case SS_MODE_OPEN_LOOP: servo_stepper_mode_open_loop(ss, position); break;
    case SS_MODE_TORQUE: servo_stepper_mode_torque_update(ss, position); break;
    case SS_MODE_HPID: servo_stepper_mode_hpid_update(ss, position); break;
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

void
command_servo_stepper_set_disabled(uint32_t *args)
{
    struct servo_stepper *ss = servo_stepper_oid_lookup(args[0]);
    irq_disable();
    ss->flags = SS_MODE_DISABLED;
    a4954_disable(ss->stepper_driver);
    irq_enable();
}
DECL_COMMAND(command_servo_stepper_set_disabled,
             "servo_stepper_set_disabled oid=%c");

void
command_servo_stepper_set_open_loop_mode(uint32_t *args)
{
    struct servo_stepper *ss = servo_stepper_oid_lookup(args[0]);
    irq_disable();
    a4954_enable(ss->stepper_driver);
    ss->flags = SS_MODE_OPEN_LOOP;
    ss->run_current_scale = args[1];
    ss->hold_current_scale = args[2];
    irq_enable();
}
DECL_COMMAND(command_servo_stepper_set_open_loop_mode,
             "servo_stepper_set_open_loop_mode oid=%c"
             " run_current_scale=%u, hold_current_scale=%u");

void
command_servo_stepper_set_hpid_mode(uint32_t *args)
{
    struct servo_stepper *ss = servo_stepper_oid_lookup(args[0]);
    if (!(ss->flags & SS_MODE_OPEN_LOOP))
        shutdown("PID mode must transition from open-loop");
    irq_disable();
    ss->flags = SS_MODE_HPID;
    uint32_t position = args[1];
    ss->pid_ctrl.last_position = position;
    virtual_stepper_set_position(ss->virtual_stepper, position);
    ss->pid_ctrl.Kp = args[2];
    ss->pid_ctrl.Ki = args[3];
    ss->pid_ctrl.Kd = args[4];
    irq_enable();
}
DECL_COMMAND(command_servo_stepper_set_hpid_mode,
             "servo_stepper_set_hpid_mode oid=%c stepper_pos=%u"
             " kp=%hi ki=%hi kd=%hi");
void
command_servo_stepper_set_torque_mode(uint32_t *args)
{
    struct servo_stepper *ss = servo_stepper_oid_lookup(args[0]);
    irq_disable();
    a4954_enable(ss->stepper_driver);
    ss->flags = SS_MODE_TORQUE;
    ss->excite_angle = args[1];
    ss->run_current_scale = args[2];
    irq_enable();
}
DECL_COMMAND(command_servo_stepper_set_torque_mode,
             "servo_stepper_set_torque_mode oid=%c"
             " excite_angle=%u current_scale=%u");
