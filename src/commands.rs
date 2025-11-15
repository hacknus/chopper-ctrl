use alloc::sync::Arc;

use arrform::{arrform, ArrForm};
use freertos_rust::*;

use crate::usb_println;

fn extract_value(cmd: &str) -> Option<f32> {
    let mut start_index = 0;
    let mut end_index = 0;
    for (i, char) in cmd.bytes().enumerate() {
        if char == b'=' {
            start_index = i + 1;
        } else if !b"-0123456789.".contains(&char) && start_index != 0 && start_index != i {
            end_index = i;
            break;
        }
    }
    if end_index == 0 {
        end_index = cmd.len();
    }
    let v = &cmd[start_index..end_index];
    match v.parse::<f32>() {
        Ok(val) => Some(val),
        Err(_) => None,
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Commands {
    SetSpeed(f32),
    SetKp(f32),
    SetKi(f32),
    SetKd(f32),
    Start,
    Stop,
}

const CMD_QUEUE_TIMEOUT: u32 = 5;

pub fn extract_command(
    cmd: &str,
    motor_command_queue: &Arc<Queue<Commands>>,
    hk: &mut bool,
    hk_period: &mut f32,
) {
    if cmd.contains("[CMD]") {
        if cmd.contains("[CMD] setSpeed=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64, "[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64, "[ACK] set speed OK, val = {}", val).as_str());
                    motor_command_queue
                        .send(Commands::SetSpeed(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setKp=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64, "[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64, "[ACK] set Kp OK, val = {}", val).as_str());
                    motor_command_queue
                        .send(Commands::SetKp(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setKi=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64, "[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64, "[ACK] set Ki OK, val = {}", val).as_str());
                    motor_command_queue
                        .send(Commands::SetKi(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setKd=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64, "[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64, "[ACK] set Kd OK, val = {}", val).as_str());
                    motor_command_queue
                        .send(Commands::SetKd(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] start") {
            motor_command_queue
                .send(Commands::Start, Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
        } else if cmd.contains("[CMD] stop") {
            motor_command_queue
                .send(Commands::Stop, Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
        } else if cmd.contains("[CMD] disableHK") {
            *hk = false;
            cmd_ok();
        } else if cmd.contains("[CMD] enableHK") {
            *hk = true;
            cmd_ok();
        } else if cmd.contains("[CMD] setHKRate") {
            match extract_value(cmd) {
                Some(r) => {
                    if r <= 2000.0 {
                        *hk_period = 1000.0 / r;
                        usb_println(arrform!(64, "[ACK] cmd OK, val = {}", r).as_str());
                    } else {
                        usb_println(
                            arrform!(64, "[ACK] error, value = {} is too large (> 2000)", r)
                                .as_str(),
                        );
                    }
                }
                None => {
                    usb_println(arrform!(64, "[ACK] error = no value found").as_str());
                }
            }
        } else {
            // invalid command
            cmd_failed();
        }
    }
}

pub fn cmd_failed() {
    usb_println("[ACK] ERR command invalid");
}

pub fn cmd_ok() {
    usb_println("[ACK] OK command valid");
}

pub fn send_housekeeping(
    enc_pos: u16,
    target: f64,
    speed: f64,
    p: f64,
    i: f64,
    d: f64,
    pwr: u16,
    msg: &str,
) {
    let hk = arrform!(
        164,
        "[HK]: {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {}",
        enc_pos,
        target,
        speed,
        p,
        i,
        d,
        pwr,
        msg
    );
    usb_println(hk.as_str());
}
