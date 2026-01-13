#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]

/************************************************************************************************************************
 * INCLUDES
 ************************************************************************************************************************/
use crate::utils;

const PID_CONTROLLER_NAME: &str = "PID Controller Crate";
const PID_CONTROLLER_RELEASE: &str = "0.1.0";
const PID_CONTROLLER_AUTHOR: &str = "R.A.G.P";
const PID_CONTROLLER_LAST_UPDATE: &str = "2026-01-13";

#[derive(Debug)]
pub struct Pid {
    //Control constants
    kp: f32,
    ki: f32,
    kd: f32,
    ksat: f32,
    //Controller limits
    up_lim: f32,
    lw_lim: f32,
    //Controller sample time
    t_s: f32,
    //Controller performance
    output: f32,
    output_sat: f32,
    prop_term_output: f32,
    int_term_output: f32,
    der_term_output: f32,
    prev_output: f32,
    prev_prop_term_output: f32,
    prev_int_term_output: f32,
    prev_der_term_output: f32,
    error: f32,
    error_summ: f32,
    prev_error: f32,
    //Controller status
}

/* 
 * Public implementations
 */
impl Pid {
    pub fn new(kp: f32, ki: f32, kd: f32, ksat: f32) -> Self {
        Self { 
            kp, 
            ki, 
            kd, 
            ksat, 
            up_lim: 0.0f32, 
            lw_lim: 0.0f32, 
            t_s: 0.0f32, 
            output: 0.0f32,
            output_sat: 0.0f32,
            prop_term_output: 0.0f32,
            int_term_output: 0.0f32,
            der_term_output: 0.0f32,
            prev_output: 0.0f32, 
            prev_prop_term_output: 0.0f32,  
            prev_int_term_output: 0.0f32,
            prev_der_term_output: 0.0f32,
            error: 0.0f32,
            error_summ: 0.0f32,  
            prev_error: 0.0f32, 
        }
    }

    pub fn set_pid_constants(&mut self, kp: f32, ki: f32, kd: f32, ksat: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
        self.ksat = ksat;
    }

    pub fn set_pid_config(&mut self, up_lim: f32, lw_lim: f32, t_s: f32) {
        self.set_pid_limits(up_lim, lw_lim);
        self.set_pid_sample_time(t_s);
    }

    pub fn pid_step_wo_aw(&mut self, setp: f32, meas: f32) -> f32 {
        //Controller error:
        let error: f32 = setp - meas;

        //Controller proportional term:
        let pid_prop_term: f32 = error * self.kp;

        //Controller integral term:
        self.prev_int_term_output += self.ki * error * self.t_s;
        let pid_int_term: f32 = self.prev_int_term_output;

        //Controller derivative term:
        let pid_der_term: f32 = ((error - self.prev_error) / self.t_s) * self.kd; 

        //Previous values update:
        self.prev_error = error;
        self.prev_prop_term_output = pid_prop_term;
        self.prev_der_term_output = pid_der_term;

        //Error summation:
        self.error_summ += error;

        //Controller output:
        self.output = pid_prop_term + pid_int_term + pid_der_term;
        self.output_sat = utils::limit(self.output, self.lw_lim, self.up_lim);

        //Controller output:
        self.output_sat
    }

    pub fn pid_step_w_aw_clamp() {
        todo!();
    }

    pub fn pid_step_w_aw_backcalc() {
        todo!();
    }

    pub fn pid_step_w_aw_condint() {
        todo!();
    }

    pub fn pid_step_w_aw_tracking() {
        todo!();
    }

    pub fn print_pid_performance(&self) {
        println!("Controller output: {} (P={}, I={}, D={})", 
        self.output, 
        self.prop_term_output,
        self.int_term_output,
        self.der_term_output
        );

        println!("Controller saturated output: {} <= {} <= {}", 
        self.lw_lim, 
        self.output_sat,
        self.up_lim);

        println!("Controller error: {}. Accumulated error: {}", 
        self.error,
        self.error_summ);
    }

    pub fn print_pid_config(&self) {
        println!("Kp={}, Ki={}, Kd={}, Ksat={}",
        self.kp,
        self.ki,
        self.kd,
        self.ksat);

        println!("Lower limit: {}, upper limit: {}. Sample time: {}s",
        self.lw_lim,
        self.up_lim,
        self.t_s);
    }

    pub fn print_pid(&self) -> () {
        self.print_pid_config();
        self.print_pid_performance();
    }
}

/*
 * Private implementations
 */
impl Pid {
    fn set_pid_limits(&mut self, up_lim: f32, lw_lim: f32) {
        self.up_lim = up_lim;
        self.lw_lim = lw_lim;
    }  

    fn set_pid_sample_time(&mut self, t_s: f32) {
        self.t_s = t_s;
    }
}

pub fn print_module_info() {
    println!("Crate name: {}", PID_CONTROLLER_NAME);
    println!("Crate release: {}", PID_CONTROLLER_RELEASE);
    println!("Crate author: {}", PID_CONTROLLER_AUTHOR);
    println!("Crate last update: {}", PID_CONTROLLER_LAST_UPDATE);
}