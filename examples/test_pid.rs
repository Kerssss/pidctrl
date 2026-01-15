/************************************************************************************************************************
 * INCLUDES
 ************************************************************************************************************************/

/************************************************************************************************************************
 * ALIAS
 ************************************************************************************************************************/
use pidctrl::pid::Pid;
use std::thread;
use std::time::Duration;

fn main() {
    //Controller config
    let t_s: f32 = 0.015f32;
    let kp: f32 = 0.3f32;
    let ki: f32 = 0.6f32;
    let kd: f32 = 0.0f32;
    let kaw: f32 = 0.0f32;
    let up_lim: f32 = 1.0f32;
    let lw_lim: f32 = -1.0f32;

    //System config
    let tau: f32 = 0.3f32;
    let k: f32 = 100.0f32;

    //Controller
    let mut controller: Pid = Pid::new(kp, ki, kd, kaw);
    controller.set_pid_config(up_lim, lw_lim, t_s);
    controller.print_pid();

    //System
    let mut system_meas = 0.0f32;

    loop {
        let actuator = controller.pid_step_wo_aw(10.0f32, system_meas);

        system_meas = system_meas + t_s * ((-(1.0f32 / tau) * system_meas) + (k / tau) * actuator);

        println!("System: {}", system_meas);
        controller.print_pid_performance();

        thread::sleep(Duration::from_millis(500));
    }
}