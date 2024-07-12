use core::marker::PhantomData;
use serde::{Deserialize, Serialize};

#[derive(Debug)]
pub struct Request<'a, T>
where
    T: Deserialize<'a>,
{
    target: &'a str,
    pub data: T,
}

impl<'a, T> Request<'a, T>
where
    T: Deserialize<'a>,
{
    pub fn target(&self) -> &'a str {
        self.target
    }
}

use serde::de::value::MapAccessDeserializer;
use serde::de::{Deserializer, MapAccess, Visitor};
impl<'a, T> Deserialize<'a> for Request<'a, T>
where
    T: Deserialize<'a> + 'a,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'a>,
    {
        struct ReqVisitor<'b, U: Deserialize<'b>> {
            lt: PhantomData<&'b U>,
        }

        impl<'de, V> Visitor<'de> for ReqVisitor<'de, V>
        where
            V: Deserialize<'de>,
        {
            type Value = Request<'de, V>;

            fn expecting(&self, _: &mut core::fmt::Formatter) -> core::fmt::Result {
                Ok(())
            }

            fn visit_map<M>(self, mut map: M) -> Result<Self::Value, M::Error>
            where
                M: MapAccess<'de>,
            {
                check_status(&mut map)?;
                let target = check_target(&mut map)?;

                let de = MapAccessDeserializer::new(map);

                let data = if core::mem::size_of::<V>() == 0 {
                    todo!()
                } else {
                    V::deserialize(de)?
                };

                Ok(Request { target, data })
            }
        }

        deserializer.deserialize_map(ReqVisitor { lt: PhantomData })
    }
}

fn check_status<'de, M>(map: &mut M) -> Result<(), M::Error>
where
    M: MapAccess<'de>,
{
    use serde::de::Error;
    match map.next_entry::<&[u8], &[u8]>()? {
        Some((b"status", b"OK")) => Ok(()),
        Some((b"status", b"Not Found")) => Err("could not find endpoint").map_err(M::Error::custom),
        o => {
            panic!("unexpected err {o:?}");
        }
    }
}

fn check_target<'de, M>(map: &mut M) -> Result<&'de str, M::Error>
where
    M: MapAccess<'de>,
{
    match map.next_entry::<&[u8], &[u8]>()? {
        Some((b"reqTarget", target)) => {
            let t = unsafe { std::str::from_utf8_unchecked(target) };
            Ok(t)
        }
        o => {
            panic!("unexpexted output: {o:?}");
        }
    }
}

pub trait ReturnVariant {
    const VARIANT: u8;
}

macro_rules! impl_variant {
    ($($t:ty => $val:expr),+,) => {
        $(
            impl ReturnVariant for $t {
                const VARIANT: u8 = $val;
            }
         )+
    };
    ($($t:ident),+,) => {
        #[allow(non_camel_case_types)]
        #[derive(PartialEq, Eq, Debug)]
        pub enum CmdKind {
            $($t),+,
        }

        impl From<u8> for CmdKind {
            fn from(num: u8) -> CmdKind {
                match num {
                    $(
                        <$t as ReturnVariant>::VARIANT => CmdKind::$t,
                     )+
                    _ => unreachable!(),
                }
            }
        }
    }
}

impl_variant! {
    Empty,
    CVP,
    ControllerConfigRaw,
    MotorErrorRaw,
    TrapezoidalTrajectory,
    MotorConfig,
    NetworkSettings,
    RootConfig,
    RootInfo,
    RequestedState,
    bool,
    IoState,
    EncoderInfo,
    AbsEncoderPos,
}

impl_variant! {
    Empty => 0,
    CVP => 1,
    ControllerConfigRaw => 2,
    MotorErrorRaw<'_> => 3,
    TrapezoidalTrajectory => 4,
    MotorConfig => 5,
    NetworkSettings<'_> => 6,
    RootConfig => 7,
    RootInfo<'_> => 8,
    RequestedState => 9,
    bool => 10,
    IoState => 11,
    EncoderInfo => 12,
    AbsEncoderPos => 13,
}

impl<T> ReturnVariant for Property<T>
where
    T: ReturnVariant,
{
    const VARIANT: u8 = T::VARIANT;
}

#[derive(Debug)]
pub struct Empty;

impl<'a> Deserialize<'a> for Empty {
    fn deserialize<D>(_: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'a>,
    {
        Ok(Empty)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxisState {
    Idle = 1,
    Enable = 8,
}

impl From<AxisState> for u8 {
    fn from(f: AxisState) -> u8 {
        match f {
            AxisState::Idle => 1,
            AxisState::Enable => 8,
        }
    }
}

impl From<u8> for AxisState {
    fn from(f: u8) -> AxisState {
        match f {
            1 => AxisState::Idle,
            8 => AxisState::Enable,
            _ => unreachable!(),
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct RequestedState {
    pub current_state: u8,
}

#[derive(Debug, Deserialize)]
pub struct Property<T> {
    pub property: T,
}

#[derive(Debug)]
pub enum ControlMode {
    Voltage = 0,
    Current = 1,
    Velocity = 2,
    Position = 3,
    Trajectory = 4,
}

impl From<ControlMode> for u8 {
    fn from(from: ControlMode) -> u8 {
        use ControlMode as CC;
        match from {
            CC::Voltage => 0,
            CC::Current => 1,
            CC::Velocity => 2,
            CC::Position => 3,
            CC::Trajectory => 4,
        }
    }
}

impl From<u8> for ControlMode {
    fn from(val: u8) -> ControlMode {
        use ControlMode as CC;
        match val {
            0 => CC::Voltage,
            1 => CC::Current,
            2 => CC::Velocity,
            3 => CC::Position,
            4 => CC::Trajectory,
            _ => unreachable!(),
        }
    }
}

/// current, velocity, and position of the motor
#[derive(Debug, Deserialize, Clone, Copy)]
pub struct CVP {
    pub position: f64,
    pub velocity: f64,
    pub current: f64,
}

#[derive(Debug, Deserialize)]
pub struct ControllerConfigRaw {
    control_mode: u8,
    input_mode: u8,
    pos_gain: f64,
    vel_gain: f64,
    vel_integrator_gain: f64,
    vel_limit: f64,
    vel_limit_tolerance: f64,
}

#[derive(Debug)]
pub struct ControllerConfig {
    pub control_mode: ControlMode,
    pub input_mode: u8,
    pub pos_gain: f64,
    pub vel_gain: f64,
    pub vel_integrator_gain: f64,
    pub vel_limit: f64,
    pub vel_limit_tolerance: f64,
}

impl From<ControllerConfigRaw> for ControllerConfig {
    fn from(from: ControllerConfigRaw) -> ControllerConfig {
        let ControllerConfigRaw {
            control_mode,
            input_mode,
            pos_gain,
            vel_gain,
            vel_integrator_gain,
            vel_limit,
            vel_limit_tolerance,
        } = from;

        ControllerConfig {
            control_mode: control_mode.into(),
            input_mode,
            pos_gain,
            vel_gain,
            vel_integrator_gain,
            vel_limit,
            vel_limit_tolerance,
        }
    }
}

#[derive(Deserialize)]
pub struct MotorErrorRaw<'a> {
    axis: &'a str,
    motor: &'a str,
    encoder: &'a str,
}

#[derive(Debug)]
pub struct MotorError<'a> {
    pub axis: Option<&'a str>,
    pub motor: Option<&'a str>,
    pub encoder: Option<&'a str>,
}

impl<'a> From<MotorErrorRaw<'a>> for MotorError<'a> {
    fn from(from: MotorErrorRaw<'a>) -> MotorError<'a> {
        let MotorErrorRaw {
            axis,
            motor,
            encoder,
        } = from;

        MotorError {
            axis: check_motor_err(axis),
            motor: check_motor_err(motor),
            encoder: check_motor_err(encoder),
        }
    }
}

fn check_motor_err<'a>(maybe_err: &'a str) -> Option<&'a str> {
    if maybe_err == "ERROR_NONE" {
        None
    } else {
        Some(maybe_err)
    }
}

#[derive(Debug, Deserialize)]
pub struct TrapezoidalTrajectory {
    pub accel_limit: f64,
    pub decel_limit: f64,
    pub vel_limit: f64,
}

#[derive(Debug, Deserialize)]
pub struct MotorConfig {
    pub current_lim: f64,
    pub current_lim_margin: f64,
    pub inverter_temp_limit_lower: f64,
    pub inverter_temp_limit_upper: f64,
    pub requested_current_range: f64,
    pub current_control_bandwidth: f64,
}

#[derive(Debug, Deserialize)]
pub struct NetworkSettings<'a> {
    #[serde(rename(deserialize = "DHCP_enable"))]
    pub dhcp_enable: bool,
    #[serde(rename(deserialize = "SSID"))]
    pub ssid: &'a str,
    pub password: &'a str,
    pub name: &'a str,
    #[serde(rename(deserialize = "staticIP"))]
    pub static_ip: [u8; 4],
    pub gateway: [u8; 4],
    pub subnet: [u8; 4],
    pub dns_1: [u8; 4],
    pub dns_2: [u8; 4],
}

#[derive(Debug, Deserialize)]
pub struct RootConfig {
    pub dc_bus_overvoltage_trip_level: f64,
    pub dc_bus_undervoltage_trip_level: f64,
}

#[derive(Debug, Deserialize)]
pub struct RootInfo<'a> {
    pub serial_number: &'a str,
    pub name: &'a str,
    pub connect_mode: &'a str,
    pub model: &'a str,
    pub motor_drive_ready: bool,
    pub gear_ratio: f64,
    pub vbus_voltage: f64,
    pub motor_temp_m1: f64,
    pub inverter_temp_m1: f64,
    #[serde(rename(deserialize = "Hw_version"))]
    pub hw_version: &'a str,
    #[serde(rename(deserialize = "Fw_version"))]
    pub fw_version: &'a str,
}

//NOTE: idk what this means, its returning null for
#[derive(Debug, Deserialize)]
pub struct IoState {}

#[derive(Debug, Deserialize)]
pub struct EncoderInfo {
    pub enc_calibrated: bool,
    pub abs_offset: f64,
    pub enc_resolution: f64,
    //TODO: fork serde to make to support const generic impl for [T; N].
    //pub offset_lut: [u8; 128],
}

#[derive(Debug, Deserialize)]
pub struct AbsEncoderPos {
    pub abs_pos: f64,
}
