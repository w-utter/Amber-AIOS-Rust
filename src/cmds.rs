use crate::serde::*;
use serde_json::json;
use serde_json::Value as JSVal;

const DATA_PORT: u16 = 2333;
const SERVICE_PORT: u16 = 2334;

macro_rules! motor_path {
    ($num:expr, $post:expr) => {
        match $num {
            0 => constcat::concat!("/m0/", $post),
            1 => constcat::concat!("/m1/", $post),
            o => panic!("invalid motor num: {o}, only 0 & 1 are supported"),
        }
    };
}

macro_rules! impl_msg {
    ($name:ident, $port:expr, $out:ty) => {
        pub struct $name {
            inner: JSVal,
        }

        impl<'readbuf> Command<'readbuf> for $name {
            const PORT: u16 = $port;
            type Return = $out;
            const RETURN_VARIANT: u8 = Self::Return::VARIANT;

            fn cmd(&self) -> &JSVal {
                &self.inner
            }
        }

        impl From<JSVal> for $name {
            fn from(inner: JSVal) -> $name {
                $name { inner }
            }
        }

        impl<'readbuf> SerializableCommand<'readbuf> for $name {
            const PORT: u16 = $port;
            type Return = $out;
            type Error = serde_json::Error;
            const RETURN_VARIANT: u8 = <Self as Command<'readbuf>>::Return::VARIANT;

            unsafe fn serialize<'b, T: AsMut<[u8]> + Sized>(
                &self,
                buf: &'b mut T,
            ) -> Result<&'b [u8], Self::Error>
            where
                Self: Sized,
            {
                crate::aios_motor::serialize_cmd(buf, self.cmd())
            }

            // NOTE: this removes the reqTarget from the request,
            // but it's never really used other than for validation so its ok to drop it here
            unsafe fn parse_return(ret: &'readbuf [u8]) -> Result<Self::Return, Self::Error> {
                let req_ret = <Self as Command<'readbuf>>::parse_return(ret)?;
                Ok(req_ret.data)
            }
        }
    };

    ($name:ident, $port:expr) => {
        impl_msg!($name, $port, Empty);
    };
}

pub trait Command<'readbuf>
where
    Self: 'readbuf,
{
    const PORT: u16;
    type Return: serde::Deserialize<'readbuf> + crate::serde::ReturnVariant;
    const RETURN_VARIANT: u8;

    fn cmd(&self) -> &JSVal;

    fn parse_return(
        bytes: &'readbuf [u8],
    ) -> Result<Request<'readbuf, Self::Return>, serde_json::Error> {
        let s = unsafe { core::str::from_utf8_unchecked(bytes) };
        let parsed: Request<<Self as Command<'readbuf>>::Return> = serde_json::from_str(s)?;

        Ok(parsed)
    }
}

impl_msg!(SetRequestedState, SERVICE_PORT, RequestedState);

pub fn set_requested_state<const MOTOR_NUM: u8>(state: AxisState) -> SetRequestedState {
    let state: u8 = state.into();
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "requested_state"),
        "property": state,
    })
    .into()
}

impl_msg!(EncoderIsReady, SERVICE_PORT, Property<bool>);

pub fn encoder_is_ready<const MOTOR_NUM: u8>() -> EncoderIsReady {
    json!({
        "method": "GET",
        "reqTarget": motor_path!(MOTOR_NUM, "encoder/is_ready"),
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

pub fn get_requested_state<const MOTOR_NUM: u8>() -> GetRequestedState {
    json!({
        "method": "GET",
        "reqTarget": motor_path!(MOTOR_NUM, "requested_state"),
    })
    .into()
}

impl_msg!(SetControlMode, SERVICE_PORT);

pub fn set_control_mode<const MOTOR_NUM: u8>(control_mode: ControlMode) -> SetControlMode {
    let ctrl: u8 = control_mode.into();
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "controller/config"),
        "control_mode": ctrl,
    })
    .into()
}

impl_msg!(SetMotionCtrlConfig, SERVICE_PORT);

pub fn set_motion_control_config<const MOTOR_NUM: u8>(
    pos_gain: f64,
    vel_gain: f64,
    vel_integrator_gain: f64,
    vel_limit: f64,
    vel_limit_tolerance: f64,
) -> SetMotionCtrlConfig {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "controller/config"),
        "pos_gain": pos_gain,
        "vel_gain": vel_gain,
        "vel_integrator_gain": vel_integrator_gain,
        "vel_limit": vel_limit,
        "vel_limit_tolerance": vel_limit_tolerance,
    })
    .into()
}

impl_msg!(SetLinearCount, SERVICE_PORT);

pub fn set_linear_count<const MOTOR_NUM: u8>(linear_count: u8) -> SetLinearCount {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "encoder"),
        "set_linear_count": linear_count,
    })
    .into()
}

impl_msg!(GetCVP, DATA_PORT, CVP);

pub fn cvp<const MOTOR_NUM: u8>() -> GetCVP {
    json!({
        "method": "GET",
        "reqTarget": motor_path!(MOTOR_NUM, "CVP"),
    })
    .into()
}

impl_msg!(GetControllerConfig, SERVICE_PORT, ControllerConfigRaw);

pub fn get_controller_config<const MOTOR_NUM: u8>() -> GetControllerConfig {
    json!({
        "method": "GET",
        "reqTarget": motor_path!(MOTOR_NUM, "controller/config"),
    })
    .into()
}

impl_msg!(GetErr, SERVICE_PORT, MotorErrorRaw<'readbuf>);

pub fn get_err<const MOTOR_NUM: u8>() -> GetErr {
    json!({
        "method": "GET",
        "reqTarget": motor_path!(MOTOR_NUM, "error"),
    })
    .into()
}

impl_msg!(ClearErr, SERVICE_PORT, MotorErrorRaw<'readbuf>);

pub fn clear_err<const MOTOR_NUM: u8>() -> ClearErr {
    json!({
        "method": "GET",
        "reqTarget": motor_path!(MOTOR_NUM, "error"),
        "clear_error": true,
    })
    .into()
}

impl_msg!(SetVelocity, DATA_PORT, CVP);

pub fn set_velocity<const MOTOR_NUM: u8>(velocity: f64, current_ff: f64) -> SetVelocity {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "setVelocity"),
        "velocity": velocity,
        "current_ff": current_ff,
    })
    .into()
}

impl_msg!(SetVelocitySilent, DATA_PORT, CVP);

pub fn set_velocity_silent<const MOTOR_NUM: u8>(
    velocity: f64,
    current_ff: f64,
) -> SetVelocitySilent {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "setVelocity"),
        "velocity": velocity,
        "current_ff": current_ff,
        "reply_enable": false,
    })
    .into()
}

impl_msg!(SetPosition, DATA_PORT, CVP);

pub fn set_position<const MOTOR_NUM: u8>(
    position: f64,
    velocity: f64,
    current_ff: f64,
) -> SetPosition {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "setPosition"),
        "velocity": velocity,
        "current_ff": current_ff,
        "position": position,
    })
    .into()
}

impl_msg!(SetPositionSilent, DATA_PORT, CVP);

pub fn set_position_silent<const MOTOR_NUM: u8>(
    position: f64,
    velocity: f64,
    current_ff: f64,
) -> SetPositionSilent {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "setPosition"),
        "velocity": velocity,
        "current_ff": current_ff,
        "position": position,
        "reply_enable": false,
    })
    .into()
}

impl_msg!(SetCurrent, DATA_PORT, CVP);

pub fn set_current<const MOTOR_NUM: u8>(current: f64) -> SetCurrent {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "setCurrent"),
        "current": current,
    })
    .into()
}

impl_msg!(SetCurrentSilent, DATA_PORT, CVP);

pub fn set_current_silent<const MOTOR_NUM: u8>(current: f64) -> SetCurrentSilent {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "setCurrent"),
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

impl_msg!(SetRootConfig, SERVICE_PORT, RootConfig);

pub fn set_root_config(
    dc_bus_overvoltage_trip_level: f64,
    dc_bus_undervoltage_trip_level: f64,
) -> SetRootConfig {
    json!({
        "method": "SET",
        "reqTarget": "/config",
        "dc_bus_overvoltage_trip_level": dc_bus_overvoltage_trip_level,
        "dc_bus_undervoltage_trip_level" : dc_bus_undervoltage_trip_level,
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

impl_msg!(SetNetworkSettings, SERVICE_PORT, NetworkSettings<'readbuf>);

pub enum NetworkOptions {
    Dhcp,
    Static {
        ip: [u8; 4],
        gateway: [u8; 4],
        subnet: [u8; 4],
        dns_1: [u8; 4],
        dns_2: [u8; 4],
    },
}

pub fn set_network_settings(
    ssid: &str,
    password: &str,
    name: &str,
    settings: NetworkOptions,
) -> SetNetworkSettings {
    match settings {
        NetworkOptions::Dhcp => {
            json!({
                "method": "SET",
                "reqTarget": "/network_setting",
                "DHCP_enable": true,
                "SSID": ssid,
                "password": password,
                "name": name,
            })
        }
        NetworkOptions::Static {
            ip,
            gateway,
            subnet,
            dns_1,
            dns_2,
        } => {
            json!({
                "method": "SET",
                "reqTarget": "/network_setting",
                "DHCP_enable": false,
                "SSID": ssid,
                "password": password,
                "name": name,
                "staticIP": ip,
                "gateway": gateway,
                "subnet": subnet,
                "dns_1": dns_1,
                "dns_2": dns_2,
            })
        }
    }
    .into()
}

impl_msg!(GetMotorConfig, SERVICE_PORT, MotorConfig);

pub fn get_motor_config<const MOTOR_NUM: u8>() -> GetMotorConfig {
    json!({
        "method": "GET",
        "reqTarget": motor_path!(MOTOR_NUM, "motor/config"),
    })
    .into()
}

impl_msg!(SetMotorConfig, SERVICE_PORT, MotorConfig);

pub fn set_motor_config<const MOTOR_NUM: u8>(
    current_lim: f64,
    current_lim_margin: f64,
    inverter_temp_limit_lower: f64,
    inverter_temp_limit_upper: f64,
    requested_current_range: f64,
    current_control_bandwidth: f64,
) -> SetMotorConfig {
    json!({
        "method" : "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "motor/config"),
        "current_lim" : current_lim,
        "current_lim_margin" : current_lim_margin,
        "inverter_temp_limit_lower" : inverter_temp_limit_lower,
        "inverter_temp_limit_upper" : inverter_temp_limit_upper,
        "requested_current_range" : requested_current_range,
        "current_control_bandwidth" : current_control_bandwidth,
    })
    .into()
}

impl_msg!(SaveConfig, SERVICE_PORT);

pub fn save_config() -> SaveConfig {
    json!({
        "method" : "SET",
        "reqTarget" : "/",
        "property" : "save_config"
    })
    .into()
}

impl_msg!(EraseConfig, SERVICE_PORT);

pub fn erase_config() -> EraseConfig {
    json!({
        "method" : "SET",
        "reqTarget" : "/",
        "property" : "erase_config"
    })
    .into()
}

impl_msg!(UpdateFirmware, SERVICE_PORT);

pub fn update_firmware() -> UpdateFirmware {
    json!({
        "method" : "SET",
        "reqTarget" : "/",
        "property" : "OTA_update"
    })
    .into()
}

impl_msg!(GetTrapTraj, SERVICE_PORT, TrapezoidalTrajectory);

pub fn get_trap_traj<const MOTOR_NUM: u8>() -> GetTrapTraj {
    json!({
        "method": "GET",
        "reqTarget": motor_path!(MOTOR_NUM, "trap_traj"),
    })
    .into()
}

impl_msg!(SetTrapTraj, SERVICE_PORT, TrapezoidalTrajectory);

pub fn set_trap_traj<const MOTOR_NUM: u8>(
    accel_limit: f64,
    decel_limit: f64,
    vel_limit: f64,
) -> SetTrapTraj {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "trap_traj"),
        "accel_limit": accel_limit,
        "devel_limit": decel_limit,
        "vel_limit": vel_limit,
    })
    .into()
}

impl_msg!(EnableVelocityRamp, SERVICE_PORT);

pub fn enable_velocity_ramp<const MOTOR_NUM: u8>(enable: bool) -> EnableVelocityRamp {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "controller"),
        "enable": enable,
    })
    .into()
}

impl_msg!(TargetVelocityRamp, DATA_PORT);

pub fn target_velocity_ramp<const MOTOR_NUM: u8>(target_velocity: f64) -> TargetVelocityRamp {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "controller"),
        "vel_ramp_target": target_velocity,
    })
    .into()
}

impl_msg!(TrapezoidalMove, DATA_PORT);

pub fn trapezoidal_move<const MOTOR_NUM: u8>(position: f64) -> TrapezoidalMove {
    json!({
        "method": "SET",
        "reqTarget": motor_path!(MOTOR_NUM, "trapezoidalMove"),
        "property": position,
    })
    .into()
}

//TODO: this is in 7DofArm/aios_python_example/aios.py
/*
impl_msg!(GetIOState, DATA_PORT, IoState);

pub fn get_io_state() -> GetIOState 
        "reqTarget": "/IO_State",
        "PWM0_CH": 0,
        "PWM1_CH": 0,
        "SERVO0": 0,
        "SERVO1": 0,
    }).into()
}
*/

impl_msg!(GetEncoderInfo, SERVICE_PORT, EncoderInfo);

pub fn get_encoder_info() -> GetEncoderInfo {
    json!({
        "method": "GET",
        "reqTarget": "/encoder_info",
    })
    .into()
}

impl_msg!(GetAbsEncoder, SERVICE_PORT, AbsEncoderPos);

pub fn absolute_encoder_position() -> GetAbsEncoder {
    json!({
        "method": "GET",
        "reqTarget": "/abs_encoder",
    })
    .into()
}

pub mod binary {
    use crate::err::{binary, Expected};
    use bincode::Options;
    const PASSTHROUGH_PORT: u16 = 10000;

    const CVP_ID: u8 = 0xFF;

    pub trait BinaryCommand<'a> {
        const MSG_ID: u8;
        type Return: serde::Deserialize<'a>;
        const PORT: u16 = PASSTHROUGH_PORT;

        unsafe fn serialize<'b>(&self, buf: &'b mut [u8]) -> &'b [u8]
        where
            Self: Sized;

        fn parse_return(ret: &'a [u8]) -> Result<Self::Return, binary::Error> {
            if ret == b"FAILED!\n" {
                return Err(binary::Error::Failed);
            } else if ret.len() < 1 + core::mem::size_of::<Self::Return>() {
                return Err(binary::Error::BadSize(Expected::new(
                    ret.len(),
                    core::mem::size_of::<Self::Return>() + 1,
                )));
            } else if ret[0] != Self::MSG_ID {
                return Err(binary::Error::BadMsgId(Expected::new(ret[0], Self::MSG_ID)));
            }

            let data = &ret[1..core::mem::size_of::<Self::Return>() + 1];
            Ok(bincode::DefaultOptions::new()
                .with_little_endian()
                .deserialize(data)
                .unwrap())
        }
    }

    macro_rules! impl_bin {
        ($name:ident, $id:expr, $in:ty) => {
            pub struct $name {
                inner: $in,
            }

            impl BinaryCommand<'_> for $name {
                const MSG_ID: u8 = $id;
                type Return = BinaryCVP;

                unsafe fn serialize<'a>(&self, buf: &'a mut [u8]) -> &'a [u8]
                where
                    Self: Sized,
                {
                    buf[0] = Self::MSG_ID;

                    let writer = std::io::BufWriter::new(&mut buf[1..]);
                    bincode::DefaultOptions::new()
                        .with_little_endian()
                        .serialize_into(writer, &self.inner)
                        .unwrap();
                    &buf[..core::mem::size_of::<Self>() + 1]
                }
            }

            impl From<$in> for $name {
                fn from(f: $in) -> $name {
                    $name { inner: f }
                }
            }

            impl<'readbuf> super::SerializableCommand<'readbuf> for $name {
                const PORT: u16 = PASSTHROUGH_PORT;
                type Return = BinaryCVP;
                type Error = binary::Error;
                const RETURN_VARIANT: u8 = $id;

                unsafe fn serialize<'b, T: AsMut<[u8]> + Sized>(
                    &self,
                    buf: &'b mut T,
                ) -> Result<&'b [u8], Self::Error>
                where
                    Self: Sized,
                {
                    Ok(<Self as BinaryCommand<'_>>::serialize(self, buf.as_mut()))
                }

                unsafe fn parse_return(ret: &'readbuf [u8]) -> Result<Self::Return, Self::Error> {
                    <Self as BinaryCommand<'_>>::parse_return(ret)
                }
            }
        };
    }

    #[derive(serde::Deserialize)]
    pub struct BinaryCVP {
        position: f32,
        velocity: f32,
        current: f32,
    }

    use crate::serde::CVP;
    impl From<BinaryCVP> for CVP {
        fn from(bin: BinaryCVP) -> CVP {
            let BinaryCVP {
                position,
                velocity,
                current,
            } = bin;

            CVP {
                position: position as _,
                velocity: velocity as _,
                current: current as _,
            }
        }
    }

    #[derive(serde::Serialize)]
    struct InputPosition {
        position: f32,
        velocity: i16,
        torque: i16,
    }

    impl_bin!(SetInputPosition, CVP_ID, InputPosition);

    pub fn set_input_position(position: f32, velocity: i16, torque: i16) -> SetInputPosition {
        InputPosition {
            position,
            velocity,
            torque,
        }
        .into()
    }

    #[derive(serde::Serialize)]
    struct InputVelocity {
        velocity: f32,
        torque: f32,
    }
    impl_bin!(SetInputVelocity, CVP_ID, InputVelocity);

    pub fn set_input_velocity(velocity: f32, torque: f32) -> SetInputVelocity {
        InputVelocity { velocity, torque }.into()
    }

    impl_bin!(SetInputTorque, CVP_ID, f32);

    pub fn set_input_torque(torque: f32) -> SetInputTorque {
        torque.into()
    }

    impl_bin!(BinGetCVP, CVP_ID, ());

    pub fn get_cvp() -> BinGetCVP {
        ().into()
    }
}

pub trait SerializableCommand<'readbuf>
where
    Self: 'readbuf,
{
    const PORT: u16;
    type Return: serde::Deserialize<'readbuf>;
    type Error: std::error::Error;
    const RETURN_VARIANT: u8;

    unsafe fn serialize<'b, T: AsMut<[u8]> + Sized>(
        &self,
        buf: &'b mut T,
    ) -> Result<&'b [u8], Self::Error>
    where
        Self: Sized;

    unsafe fn parse_return(ret: &'readbuf [u8]) -> Result<Self::Return, Self::Error>;
}
