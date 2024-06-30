use crate::serde::*;
use serde_json::json;
use serde_json::Value as JSVal;

pub fn set_requested_state(state: AxisState) -> JSVal {
    let state: u8 = state.into();
    json!({
        "method": "SET",
        "reqTarget": "/m1/requested_state",
        "property": state,
    })
}

pub fn encoder_is_ready() -> JSVal {
    json!({
        "method": "GET",
        "reqTarget": "/m1/encoder/is_ready",
    })
}

pub fn reboot() -> JSVal {
    json!({
        "method": "SET",
        "reqTarget": "/",
        "property": "reboot",

    })
}

pub fn reboot_motor_drive() -> JSVal {
    json!({
        "method": "SET",
        "reqTarget": "/",
        "property": "reboot_motor_drive",
    })
}

pub fn get_requested_state() -> JSVal {
    json!({
        "method": "GET",
        "reqTarget": "/m1/requested_state",
    })
}

pub fn set_control_mode(control_mode: ControlMode) -> JSVal {
    let ctrl: u8 = control_mode.into();
    json!({
        "method": "SET",
        "reqTarget": "/m1/controller/config",
        "control_mode": ctrl,
    })
}

pub fn set_linear_count(linear_count: u8) -> JSVal {
    json!({
        "method": "SET",
        "reqTarget": "/m1/encoder",
        "set_linear_count": linear_count,
    })
}

pub fn cvp() -> JSVal {
    json!({
        "method": "GET",
        "reqTarget": "/m1/CVP",
    })
}

pub fn get_controller_config() -> JSVal {
    json!({
        "method": "GET",
        "reqTarget": "/m1/controller/config",
    })
}

pub fn get_err() -> JSVal {
    json!({
        "method": "GET",
        "reqTarget": "/m1/error",
    })
}

pub fn clear_err() -> JSVal {
    json!({
        "method": "GET",
        "reqTarget": "/m1/error",
        "clear_error": true,
    })
}

pub fn set_velocity(velocity: f64, current_ff: f64) -> JSVal {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setVelocity",
        "velocity": velocity,
        "current_ff": current_ff,
    })
}

pub fn set_velocity_silent(velocity: f64, current_ff: f64) -> JSVal {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setVelocity",
        "velocity": velocity,
        "current_ff": current_ff,
        "reply_enable": false,
    })
}

pub fn set_position(position: f64, velocity: f64, current_ff: f64) -> JSVal {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setPosition",
        "velocity": velocity,
        "current_ff": current_ff,
        "position": position,
    })
}

pub fn set_position_silent(position: f64, velocity: f64, current_ff: f64) -> JSVal {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setPosition",
        "velocity": velocity,
        "current_ff": current_ff,
        "position": position,
        "reply_enable": false,
    })
}

pub fn set_current(current: f64) -> JSVal {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setCurrent",
        "current": current,
    })
}

pub fn set_current_silent(current: f64) -> JSVal {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setCurrent",
        "current": current,
        "reply_enable": false,
    })
}

pub fn get_root() -> JSVal {
    json!({
        "method": "GET",
        "reqTarget": "/",
    })
}

pub fn get_root_config() -> JSVal {
    json!({
        "method": "GET",
        "reqTarget": "/config",
    })
}

pub fn get_network_settings() -> JSVal {
    json!({
        "method": "GET",
        "reqTarget": "/network_setting",
    })
}

pub fn get_m1_motor_config() -> JSVal {
    json!({
        "method": "GET",
        "reqTarget": "/m1/motor/config",
    })
}

pub fn get_m1_trap_traj() -> JSVal {
    json!({
        "method": "GET",
        "reqTarget": "/m1/trap_traj",
    })
}
