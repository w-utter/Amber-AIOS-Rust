use crate::cmds;
use crate::serde::*;
use crate::socket::Socket;
use nix::Result as Res;
use serde::Deserialize;
use serde_json::Value as JSVal;
use std::net::Ipv4Addr;

pub struct AiosMotor<const R: usize, const W: usize> {
    socket: Socket<R>,
    write_buf: [u8; W],
}

struct SerCounter<'a> {
    count: usize,
    writer: std::io::BufWriter<&'a mut [u8]>,
}

impl<'a> SerCounter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        let writer = std::io::BufWriter::new(buf);

        Self { count: 0, writer }
    }
}

impl<'a> std::io::Write for SerCounter<'a> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let len = self.writer.write(buf)?;
        self.count += len;
        Ok(len)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()
    }
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

    pub unsafe fn send_recv_parse<'a, C>(
        &'a mut self,
        cmd: &C,
    ) -> Result<Request<<C as cmds::Command>::Return>, Err>
    where
        C: cmds::Command<'a>,
        <C as cmds::Command<'a>>::Return: Deserialize<'a> + 'a,
    {
        let bytes = self.send_recv(cmd.cmd(), <C as cmds::Command>::PORT)?;
        let data = <C as cmds::Command<'a>>::parse_return(bytes)?;
        Ok(data)
    }

    pub fn set_axis_state(&mut self, state: AxisState) -> Result<(), Err> {
        let cmd = cmds::set_requested_state(state);
        let data = unsafe { self.send_recv_parse(&cmd)? };
        let ret_state: AxisState = data.data.current_state.into();

        if ret_state != state {
            return Err(Err::UnexpectedReturn);
        }
        Ok(())
    }

    pub fn enable(&mut self) -> Result<(), Err> {
        self.set_axis_state(AxisState::Enable)
    }

    pub fn disable(&mut self) -> Result<(), Err> {
        self.set_axis_state(AxisState::Idle)
    }

    pub fn is_encoder_ready(&mut self) -> Result<bool, Err> {
        let cmd = cmds::encoder_is_ready();
        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data.property)
    }

    pub fn initalize(&mut self) -> Result<(), Err> {
        while !self.is_encoder_ready()? {}

        self.enable()?;
        Ok(())
    }

    pub fn shutdown(&mut self) -> Result<(), Err> {
        self.disable()
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

    pub fn get_state(&mut self) -> Result<RequestedState, Err> {
        let cmd = cmds::get_requested_state();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn set_control_mode(&mut self, control_mode: ControlMode) -> Result<(), Err> {
        let cmd = cmds::set_control_mode(control_mode);

        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn set_linear_count(&mut self, linear_count: u8) -> Result<(), Err> {
        let cmd = cmds::set_linear_count(linear_count);

        let _ = unsafe { self.send_recv_parse(&cmd)? };
        Ok(())
    }

    pub fn current_velocity_position(&mut self) -> Result<CVP, Err> {
        let cmd = cmds::cvp();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn controller_config(&mut self) -> Result<ControllerConfig, Err> {
        let cmd = cmds::get_controller_config();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data.into())
    }

    pub fn err(&mut self) -> Result<MotorError, Err> {
        let cmd = cmds::get_err();
        let data = unsafe { self.send_recv_parse(&cmd)? };
        let data: MotorError = data.data.into();
        Ok(data)
    }

    pub fn clear_err(&mut self) -> Result<MotorError, Err> {
        let cmd = cmds::clear_err();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        let data: MotorError = data.data.into();
        Ok(data)
    }

    pub fn set_velocity(&mut self, velocity: f64, current_ff: f64) -> Result<CVP, Err> {
        let cmd = cmds::set_velocity(velocity, current_ff);

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn set_position(
        &mut self,
        position: f64,
        velocity: f64,
        current_ff: f64,
    ) -> Result<CVP, Err> {
        let cmd = cmds::set_position(position, velocity, current_ff);

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn set_current(&mut self, current: f64) -> Result<CVP, Err> {
        let cmd = cmds::set_current(current);

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn idle(&mut self) -> Result<(), Err> {
        let _ = self.set_velocity(0.0, 0.0)?;
        Ok(())
    }

    pub fn get_root(&mut self) -> Result<RootInfo, Err> {
        let cmd = cmds::get_root();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    //FIXME: theres also set, save, and erase config
    //which have not been implemented
    pub fn get_root_config(&mut self) -> Result<RootConfig, Err> {
        let cmd = cmds::get_root_config();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    pub fn get_network_settings(&mut self) -> Result<NetworkSettings, Err> {
        let cmd = cmds::get_network_settings();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    //FIXME: theres also a set motor config
    //which has not been implemented
    pub fn motor_config(&mut self) -> Result<MotorConfig, Err> {
        let cmd = cmds::get_m1_motor_config();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
    }

    //FIXME: this also has a set
    //which has not been implemented
    pub fn trapezoidal_trajectory(&mut self) -> Result<TrapezoidalTrajectory, Err> {
        let cmd = cmds::get_m1_trap_traj();

        let data = unsafe { self.send_recv_parse(&cmd)? };
        Ok(data.data)
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

    pub fn serialize_cmd<'a>(&'a mut self, val: &JSVal) -> Result<&'a [u8], serde_json::Error> {
        serialize_cmd(&mut self.write_buf, val)
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
        let _ = self.disable();
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

fn serialize_cmd<'a>(buf: &'a mut [u8], val: &JSVal) -> Result<&'a [u8], serde_json::Error> {
    let writer = SerCounter::new(buf);
    use serde::Serialize;
    let mut ser = serde_json::Serializer::new(writer);

    val.serialize(&mut ser)?;

    let len = ser.into_inner().count;
    let ptr = buf.as_ptr();

    let data = unsafe { core::slice::from_raw_parts(ptr, len) };

    Ok(data)
}
