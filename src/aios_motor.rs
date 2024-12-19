use crate::cmds;
use crate::serde::*;
use crate::socket::Socket;
use nix::Result as Res;
use os_socketaddr::OsSocketAddr;
use serde::Deserialize;
use serde_json::Value as JSVal;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};

pub struct AiosMotor<const R: usize, const W: usize> {
    socket: Socket<R>,
    write_buf: [u8; W],
}

impl<const R: usize, const W: usize> AiosMotor<R, W> {
    pub const SERVICE_PORT: u16 = 2334;
    pub const DATA_PORT: u16 = 2333;

    pub fn from_socket(socket: Socket<R>) -> Self {
        let write_buf = [0u8; W];
        Self { write_buf, socket }
    }

    pub fn from_ip(a: u8, b: u8, c: u8, d: u8) -> Res<Self> {
        Ok(Self::from_socket(Socket::new(a, b, c, d)?))
    }

    pub fn from_addr(addr: Ipv4Addr) -> Res<Self> {
        Ok(Self::from_socket(Socket::from_ipv4(addr)?))
    }

    pub fn from_octets([a, b, c, d]: [u8; 4]) -> Res<Self> {
        Self::from_ip(a, b, c, d)
    }

    pub fn send_recv<'a>(&'a mut self, value: &JSVal, port: u16) -> Result<&'a [u8], Err> {
        let bytes = serialize_cmd(&mut self.write_buf, value)?;
        self.socket.send_raw(bytes, port)?;
        Ok(self.socket.recv_raw()?)
    }

    pub fn addr(&self) -> Ipv4Addr {
        self.socket.addr()
    }

    pub fn sock_addr(&self, port: u16) -> OsSocketAddr {
        let socket_addr = SocketAddr::new(IpAddr::V4(self.addr()), port);
        socket_addr.into()
    }

    pub unsafe fn send_recv_parse<'a, C>(
        &'a mut self,
        cmd: &C,
    ) -> Result<Request<'a, <C as cmds::Command<'a>>::Return>, Err>
    where
        C: cmds::Command<'a>,
        <C as cmds::Command<'a>>::Return: Deserialize<'a> + 'a,
    {
        let bytes = self.send_recv(cmd.cmd(), <C as cmds::Command>::PORT)?;
        let data = <C as cmds::Command<'a>>::parse_return(bytes)?;
        Ok(data)
    }

    pub unsafe fn send_recv_parse_bin<'a, C>(&'a mut self, cmd: &C) -> Result<C::Return, Err>
    where
        C: cmds::binary::BinaryCommand<'a>,
        <C as cmds::binary::BinaryCommand<'a>>::Return: Deserialize<'a> + 'a,
    {
        let bytes = cmd.serialize(&mut self.write_buf);
        self.socket.send_raw(bytes, C::PORT)?;
        let ret = self.socket.recv_raw()?;
        let parsed = <C as cmds::binary::BinaryCommand<'a>>::parse_return(ret);
        Ok(parsed)
    }

    pub unsafe fn send_recv_bin<'a>(
        &'a mut self,
        bytes: &[u8],
        port: u16,
    ) -> Result<&'a [u8], Err> {
        self.socket.send_raw(bytes, port)?;
        Ok(self.socket.recv_raw()?)
    }

    pub fn set_axis_state<const MN: u8>(&mut self, state: AxisState) -> Result<(), Err> {
        let cmd = cmds::set_requested_state::<MN>(state);
        let data = unsafe { self.send_recv_parse(&cmd)? };
        let ret_state: AxisState = data.data.current_state.into();

        if ret_state != state {
            return Err(Err::UnexpectedReturn);
        }
        Ok(())
    }

    pub fn enable<const MN: u8>(&mut self) -> Result<(), Err> {
        self.set_axis_state::<MN>(AxisState::Enable)
    }

    pub fn disable<const MN: u8>(&mut self) -> Result<(), Err> {
        self.set_axis_state::<MN>(AxisState::Idle)
    }

    pub fn is_encoder_ready<const MN: u8>(&mut self) -> Result<bool, Err> {
        let cmd = cmds::encoder_is_ready::<MN>();
        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data.property)
    }

    pub fn initalize<const MN: u8>(&mut self) -> Result<(), Err> {
        while !self.is_encoder_ready::<MN>()? {}

        self.enable::<MN>()?;
        Ok(())
    }

    pub fn shutdown<const MN: u8>(&mut self) -> Result<(), Err> {
        self.disable::<MN>()
    }
    //NOTE: i would not use this unless absolutely necessary
    pub fn reboot(&mut self) -> Result<(), Err> {
        let cmd = cmds::reboot();
        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    // NOTE: i would not use this unless absolutely necessary
    // this will not do anything until `reboot` is called
    // and even then i(will)dk if that fixes it completely.
    pub unsafe fn reboot_motor_drive(&mut self) -> Result<(), Err> {
        let cmd = cmds::reboot_motor_drive();

        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn complete_reboot(&mut self) -> Result<(), Err> {
        unsafe { self.reboot_motor_drive()? };
        self.reboot()?;
        Ok(())
    }

    pub fn get_state<const MN: u8>(&mut self) -> Result<RequestedState, Err> {
        let cmd = cmds::get_requested_state::<MN>();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn set_control_mode<const MN: u8>(&mut self, control_mode: ControlMode) -> Result<(), Err> {
        let cmd = cmds::set_control_mode::<MN>(control_mode);

        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn set_linear_count<const MN: u8>(&mut self, linear_count: u8) -> Result<(), Err> {
        let cmd = cmds::set_linear_count::<MN>(linear_count);

        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn current_velocity_position<const MN: u8>(&mut self) -> Result<CVP, Err> {
        let cmd = cmds::cvp::<MN>();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn controller_config<const MN: u8>(&mut self) -> Result<ControllerConfig, Err> {
        let cmd = cmds::get_controller_config::<MN>();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data.into())
    }

    pub fn set_controller_motion_config<const MN: u8>(
        &mut self,
        pos_gain: f64,
        vel_gain: f64,
        vel_integrator_gain: f64,
        vel_limit: f64,
        vel_limit_tolerance: f64,
    ) -> Result<(), Err> {
        let cmd = cmds::set_motion_control_config::<MN>(
            pos_gain,
            vel_gain,
            vel_integrator_gain,
            vel_limit,
            vel_limit_tolerance,
        );
        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn err<const MN: u8>(&mut self) -> Result<MotorError, Err> {
        let cmd = cmds::get_err::<MN>();
        let data = unsafe { self.send_recv_parse(&cmd)? };
        let data: MotorError = data.data.into();
        Ok(data)
    }

    pub fn clear_err<const MN: u8>(&mut self) -> Result<MotorError, Err> {
        let cmd = cmds::clear_err::<MN>();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        let data: MotorError = data.data.into();
        Ok(data)
    }

    pub fn set_velocity<const MN: u8>(
        &mut self,
        velocity: f64,
        current_ff: f64,
    ) -> Result<CVP, Err> {
        let cmd = cmds::set_velocity::<MN>(velocity, current_ff);

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn set_position<const MN: u8>(
        &mut self,
        position: f64,
        velocity: f64,
        current_ff: f64,
    ) -> Result<CVP, Err> {
        let cmd = cmds::set_position::<MN>(position, velocity, current_ff);

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn set_current<const MN: u8>(&mut self, current: f64) -> Result<CVP, Err> {
        let cmd = cmds::set_current::<MN>(current);

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn idle<const MN: u8>(&mut self) -> Result<(), Err> {
        let _ = self.set_velocity::<MN>(0.0, 0.0)?;
        Ok(())
    }

    pub fn get_root(&mut self) -> Result<RootInfo, Err> {
        let cmd = cmds::get_root();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn get_root_config(&mut self) -> Result<RootConfig, Err> {
        let cmd = cmds::get_root_config();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn set_root_config(
        &mut self,
        dc_bus_overvoltage_trip_level: f64,
        dc_bus_undervoltage_trip_level: f64,
    ) -> Result<RootConfig, Err> {
        let cmd = cmds::set_root_config(
            dc_bus_overvoltage_trip_level,
            dc_bus_undervoltage_trip_level,
        );

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn get_network_settings(&mut self) -> Result<NetworkSettings, Err> {
        let cmd = cmds::get_network_settings();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn set_network_settings(
        &mut self,
        ssid: &str,
        password: &str,
        name: &str,
        settings: cmds::NetworkOptions,
    ) -> Result<NetworkSettings, Err> {
        let cmd = cmds::set_network_settings(ssid, password, name, settings);

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn motor_config<const MN: u8>(&mut self) -> Result<MotorConfig, Err> {
        let cmd = cmds::get_motor_config::<MN>();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn set_motor_config<const MN: u8>(
        &mut self,
        current_lim: f64,
        current_lim_margin: f64,
        inverter_temp_limit_lower: f64,
        inverter_temp_limit_upper: f64,
        requested_current_range: f64,
        current_control_bandwidth: f64,
    ) -> Result<MotorConfig, Err> {
        let cmd = cmds::set_motor_config::<MN>(
            current_lim,
            current_lim_margin,
            inverter_temp_limit_lower,
            inverter_temp_limit_upper,
            requested_current_range,
            current_control_bandwidth,
        );
        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn save_config(&mut self) -> Result<(), Err> {
        let cmd = cmds::save_config();
        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn erase_config(&mut self) -> Result<(), Err> {
        let cmd = cmds::erase_config();
        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn update_firmware(&mut self) -> Result<(), Err> {
        let cmd = cmds::update_firmware();
        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn trapezoidal_trajectory<const MN: u8>(&mut self) -> Result<TrapezoidalTrajectory, Err> {
        let cmd = cmds::get_trap_traj::<MN>();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn set_trapezoidal_trajectory<const MN: u8>(
        &mut self,
        accel_lim: f64,
        decel_lim: f64,
        vel_lim: f64,
    ) -> Result<TrapezoidalTrajectory, Err> {
        let cmd = cmds::set_trap_traj::<MN>(accel_lim, decel_lim, vel_lim);
        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn enable_velocity_ramp<const MN: u8>(&mut self, enable: bool) -> Result<(), Err> {
        let cmd = cmds::enable_velocity_ramp::<MN>(enable);
        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn target_velocity_ramp<const MN: u8>(&mut self, target_vel: f64) -> Result<(), Err> {
        let cmd = cmds::target_velocity_ramp::<MN>(target_vel);
        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn trapezoidal_move<const MN: u8>(&mut self, position: f64) -> Result<(), Err> {
        let cmd = cmds::trapezoidal_move::<MN>(position);
        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn encoder_info(&mut self) -> Result<EncoderInfo, Err> {
        let cmd = cmds::get_encoder_info();
        let info = unsafe { self.send_recv_parse(&cmd)? };
        Ok(info.data)
    }

    pub fn encoder_absolute_position(&mut self) -> Result<f64, Err> {
        let cmd = cmds::absolute_encoder_position();
        let pos = unsafe { self.send_recv_parse(&cmd)? };
        Ok(pos.data.abs_pos)
    }

    pub fn set_input_position(
        &mut self,
        position: f32,
        velocity: i16,
        torque: i16,
    ) -> Result<CVP, Err> {
        let cmd = cmds::binary::set_input_position(position, velocity, torque);
        let cvp = unsafe { self.send_recv_parse_bin(&cmd)? };
        Ok(cvp.into())
    }

    pub fn set_input_velocity(&mut self, velocity: f32, torque: f32) -> Result<CVP, Err> {
        let cmd = cmds::binary::set_input_velocity(velocity, torque);
        let cvp = unsafe { self.send_recv_parse_bin(&cmd)? };
        Ok(cvp.into())
    }

    pub fn set_input_torque(&mut self, torque: f32) -> Result<CVP, Err> {
        let cmd = cmds::binary::set_input_torque(torque);
        let cvp = unsafe { self.send_recv_parse_bin(&cmd)? };
        Ok(cvp.into())
    }

    pub fn get_cvp(&mut self) -> Result<CVP, Err> {
        let cmd = cmds::binary::get_cvp();
        let cvp = unsafe { self.send_recv_parse_bin(&cmd)? };
        Ok(cvp.into())
    }

    /// all this does is print out the response from the given endpoint and port
    pub fn test_endpoint(&mut self, ep: &str, port: u16) -> Result<(), Err> {
        let cmd = serde_json::json!({
            "method": "GET",
            "reqTarget": ep,
        });

        let bytes = self.send_recv(&cmd, port)?;

        let s = unsafe { std::str::from_utf8_unchecked(bytes) };
        println!("\n{s}\n");
        Ok(())
    }

    pub fn serialize_json_cmd<'a>(
        &'a mut self,
        val: &JSVal,
    ) -> Result<&'a [u8], serde_json::Error> {
        serialize_cmd(&mut self.write_buf, val)
    }

    pub fn serialize_bin_cmd<'a, 'b>(
        &'a mut self,
        val: &impl cmds::binary::BinaryCommand<'b>,
    ) -> &'a [u8] {
        unsafe { val.serialize(&mut self.write_buf) }
    }

    pub fn serialize_cmd<'a, 'b, C: cmds::SerializableCommand<'b>>(
        &'a mut self,
        cmd: &C,
    ) -> Result<&'a [u8], C::Error> {
        unsafe { cmd.serialize(&mut self.write_buf) }
    }

    pub fn read_buf_mut(&mut self) -> &mut [u8] {
        self.socket.read_buf_mut()
    }

    pub fn read_buf(&self) -> &[u8] {
        self.socket.read_buf()
    }
}

impl<const R: usize, const W: usize> std::os::fd::AsRawFd for AiosMotor<R, W> {
    fn as_raw_fd(&self) -> std::os::fd::RawFd {
        self.socket.as_raw_fd()
    }
}

impl<const R: usize, const W: usize> Drop for AiosMotor<R, W> {
    fn drop(&mut self) {
        let _ = self.disable::<0>();
        let _ = self.disable::<1>();
    }
}

impl From<serde_json::Error> for Err {
    fn from(err: serde_json::Error) -> Err {
        Err::Serde(err)
    }
}

impl From<nix::Error> for Err {
    fn from(err: nix::Error) -> Err {
        Err::Io(err)
    }
}

pub enum Err {
    Serde(serde_json::Error),
    Io(nix::Error),
    UnexpectedReturn,
}

pub(crate) fn serialize_cmd<'a, T: AsMut<[u8]> + Sized>(
    buf: &'a mut T,
    val: &JSVal,
) -> Result<&'a [u8], serde_json::Error> {
    let writer = sized_writer::SizedWriter::from_borrowed(buf);
    use serde::Serialize;
    let mut ser = serde_json::Serializer::new(writer);

    val.serialize(&mut ser)?;

    let writer = ser.into_inner();

    Ok(writer.finish())
}
