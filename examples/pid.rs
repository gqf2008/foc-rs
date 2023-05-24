use chrono::Utc;
use foc_rs::fpid;
use foc_rs::pid;
use foc_rs::*;

// pid_velocity: Pid::new(
//     DEF_PID_VEL_P,
//     DEF_PID_VEL_I,
//     DEF_PID_VEL_D,
//     DEF_PID_VEL_RAMP,
//     DEF_PID_VEL_LIMIT,
// ),
// pid_angle: Pid::new(DEF_P_ANGLE_P, 0., 0., 0., DEF_VEL_LIM),
fn main() {
    let mut setpoint = 0.;
    let measurement = 3.;
    let mut pid_velocity1 = fpid::Pid::new(20., 0., 0., 0., 40.);
    let mut pid_velocity2 = pid::Pid::new(20., 0., 0.)
        .ramp(0.)
        .limit_i(40.)
        .limit_out(40.);
    (0..1000).for_each(|_| {
        let timestamp_now = Utc::now().timestamp_micros();
        let out = pid_velocity1.output(setpoint - measurement, timestamp_now);

        let out1 = pid_velocity2
            .set_point(setpoint)
            .update(measurement, timestamp_now)
            .output
            .pd();
        println!("{out} {out1}");
    });
}
