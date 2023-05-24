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
    let measurement = -2.951379;
    let mut pid_velocity1 = fpid::Pid::new(
        DEF_PID_VEL_P,
        DEF_PID_VEL_I,
        DEF_PID_VEL_D,
        DEF_PID_VEL_RAMP,
        6.,
    );
    let mut pid_velocity2 = pid::Pid::new(DEF_PID_VEL_P, DEF_PID_VEL_I, DEF_PID_VEL_D)
        .ramp(DEF_PID_VEL_RAMP)
        .limit_i(6.)
        .limit_out(6.);
    (0..1000).for_each(|_| {
        let timestamp_now = Utc::now().timestamp_micros();
        let out = pid_velocity1.output(40. - measurement, timestamp_now);

        let out1 = pid_velocity2
            .set_point(40.)
            .update(measurement, timestamp_now)
            .output();
        println!("{out} {out1}");
    });
}
