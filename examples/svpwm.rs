use std::{fmt::Write, println};

use foc_rs::*;

fn main() {
    println!("================svpwm===============");
    let mut csv1 = String::new();
    (0..=100).for_each(|i| {
        let angle_el = _3PI_2 + _2PI * i as f32 / 100.;

        let out = Voltage::Dq(0., 6.).svpwm_simplefoc(angle_el, 12.);

        csv1.write_fmt(format_args!(
            "6.0,{},{},{},{}\n",
            angle_el,
            out.Ua(),
            out.Ub(),
            out.Uc()
        ))
        .ok();
    });
    std::fs::write("svpwm.csv", csv1).ok();
    // let mut csv2 = String::new();
    // println!("================spwm===============");
    // (0..=500).for_each(|i| {
    //     let angle_el = _3PI_2 + _2PI * i as f32 / 500.;
    //     let out = Voltage::Dq(0., 6.).spwm(angle_el, 12.);
    //     csv2.write_fmt(format_args!("{},{},{}\n", out.Ua(), out.Ub(), out.Uc()))
    //         .ok();
    // });
    // std::fs::write("spwm.csv", csv2).ok();
}
