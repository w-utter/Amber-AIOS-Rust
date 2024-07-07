use crate::serde::*;
use serde_json::json;
use serde_json::Value as JSVal;

const DATA_PORT: u16 = 2333;
const SERVICE_PORT: u16 = 2334;

macro_rules! impl_msg {
    ($name:ident, $port:expr, $out:ty) => {
        pub struct $name {
            inner: JSVal,
        }

        impl<'readbuf> Command<'readbuf> for $name {
            const PORT: u16 = $port;
            type Return = $out;

            fn cmd(&self) -> &JSVal {
                &self.inner
            }
        }

        impl From<JSVal> for $name {
            fn from(inner: JSVal) -> $name {
                $name { inner }
            }
        }
    };

    ($name:ident, $port:expr) => {
        impl_msg!($name, $port, Empty<'readbuf>);
    };
}

pub trait Command<'readbuf> where Self: 'readbuf {
    const PORT: u16;
    type Return: serde::Deserialize<'readbuf>;

    fn cmd(&self) -> &JSVal;

    fn parse_return(bytes: &'readbuf [u8]) ->  Result<Request<'readbuf, Self::Return>, serde_json::Error> {
        let s = unsafe {
            core::str::from_utf8_unchecked(bytes)
        };
        let parsed: Request<<Self as Command<'readbuf>>::Return> = serde_json::from_str(s)?;
        
        Ok(parsed)
    }
}

impl_msg!(SetRequestedState, SERVICE_PORT, RequestedState);

pub fn set_requested_state(state: AxisState) -> SetRequestedState {
    let state: u8 = state.into();
    json!({
        "method": "SET",
        "reqTarget": "/m1/requested_state",
        "property": state,
    })
    .into()
}

impl_msg!(EncoderIsReady, SERVICE_PORT, Property<bool>);

pub fn encoder_is_ready() -> EncoderIsReady {
    json!({
        "method": "GET",
        "reqTarget": "/m1/encoder/is_ready",
    })
    .into()
}

impl_msg!(Reboot, SERVICE_PORT);

pub fn reboot() -> Reboot {
    json!({
        "method": "SET",
        "reqTarget": "/",
        "property": "reboot",

    })
    .into()
}

impl_msg!(RebootMotorDrive, SERVICE_PORT);

pub fn reboot_motor_drive() -> RebootMotorDrive {
    json!({
        "method": "SET",
        "reqTarget": "/",
        "property": "reboot_motor_drive",
    })
    .into()
}

impl_msg!(GetRequestedState, SERVICE_PORT, RequestedState);

pub fn get_requested_state() -> GetRequestedState {
    json!({
        "method": "GET",
        "reqTarget": "/m1/requested_state",
    })
    .into()
}

impl_msg!(SetControlMode, SERVICE_PORT);

pub fn set_control_mode(control_mode: ControlMode) -> SetControlMode {
    let ctrl: u8 = control_mode.into();
    json!({
        "method": "SET",
        "reqTarget": "/m1/controller/config",
        "control_mode": ctrl,
    })
    .into()
}

impl_msg!(SetLinearCount, SERVICE_PORT);

pub fn set_linear_count(linear_count: u8) -> SetLinearCount {
    json!({
        "method": "SET",
        "reqTarget": "/m1/encoder",
        "set_linear_count": linear_count,
    })
    .into()
}

impl_msg!(GetCVP, DATA_PORT, CVP);

pub fn cvp() -> GetCVP {
    json!({
        "method": "GET",
        "reqTarget": "/m1/CVP",
    })
    .into()
}

impl_msg!(GetControllerConfig, SERVICE_PORT, ControllerConfigRaw);

pub fn get_controller_config() -> GetControllerConfig {
    json!({
        "method": "GET",
        "reqTarget": "/m1/controller/config",
    })
    .into()
}

impl_msg!(GetErr, SERVICE_PORT, MotorErrorRaw<'readbuf>);

pub fn get_err() -> GetErr {
    json!({
        "method": "GET",
        "reqTarget": "/m1/error",
    })
    .into()
}

impl_msg!(ClearErr, SERVICE_PORT, MotorErrorRaw<'readbuf>);

pub fn clear_err() -> ClearErr {
    json!({
        "method": "GET",
        "reqTarget": "/m1/error",
        "clear_error": true,
    })
    .into()
}

impl_msg!(SetVelocity, DATA_PORT, CVP);

pub fn set_velocity(velocity: f64, current_ff: f64) -> SetVelocity {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setVelocity",
        "velocity": velocity,
        "current_ff": current_ff,
    })
    .into()
}

impl_msg!(SetVelocitySilent, DATA_PORT, CVP);

pub fn set_velocity_silent(velocity: f64, current_ff: f64) -> SetVelocitySilent {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setVelocity",
        "velocity": velocity,
        "current_ff": current_ff,
        "reply_enable": false,
    })
    .into()
}

impl_msg!(SetPosition, DATA_PORT, CVP);

pub fn set_position(position: f64, velocity: f64, current_ff: f64) -> SetPosition {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setPosition",
        "velocity": velocity,
        "current_ff": current_ff,
        "position": position,
    })
    .into()
}

impl_msg!(SetPositionSilent, DATA_PORT, CVP);

pub fn set_position_silent(position: f64, velocity: f64, current_ff: f64) -> SetPositionSilent {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setPosition",
        "velocity": velocity,
        "current_ff": current_ff,
        "position": position,
        "reply_enable": false,
    })
    .into()
}

impl_msg!(SetCurrent, DATA_PORT, CVP);

pub fn set_current(current: f64) -> SetCurrent {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setCurrent",
        "current": current,
    })
    .into()
}

impl_msg!(SetCurrentSilent, DATA_PORT, CVP);

pub fn set_current_silent(current: f64) -> SetCurrentSilent {
    json!({
        "method": "SET",
        "reqTarget": "/m1/setCurrent",
        "current": current,
        "reply_enable": false,
    })
    .into()
}

impl_msg!(GetRoot, SERVICE_PORT, RootInfo<'readbuf>);

pub fn get_root() -> GetRoot {
    json!({
        "method": "GET",
        "reqTarget": "/",
    })
    .into()
}

impl_msg!(GetRootConfig, SERVICE_PORT, RootConfig);

pub fn get_root_config() -> GetRootConfig {
    json!({
        "method": "GET",
        "reqTarget": "/config",
    })
    .into()
}

impl_msg!(GetNetworkSettings, SERVICE_PORT, NetworkSettings<'readbuf>);

pub fn get_network_settings() -> GetNetworkSettings {
    json!({
        "method": "GET",
        "reqTarget": "/network_setting",
    })
    .into()
}

impl_msg!(GetMotorConfig, SERVICE_PORT, MotorConfig);

pub fn get_m1_motor_config() -> GetMotorConfig {
    json!({
        "method": "GET",
        "reqTarget": "/m1/motor/config",
    })
    .into()
}

impl_msg!(GetTrapTraj, SERVICE_PORT, TrapezoidalTrajectory);

pub fn get_m1_trap_traj() -> GetTrapTraj {
    json!({
        "method": "GET",
        "reqTarget": "/m1/trap_traj",
    })
    .into()
}
