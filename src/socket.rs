use std::os::fd::{OwnedFd, AsRawFd, RawFd};
use nix::sys::socket::{SockaddrIn, MsgFlags};
use std::net::Ipv4Addr;
pub struct Socket<const S: usize> {
    socket: OwnedFd,
    read_buf: [u8; S],
    addr: Ipv4Addr,
}

use nix::Result as Res;
use std::time::Duration;

impl <const S: usize> Socket<S> {
    pub fn new(a: u8, b: u8, c: u8, d: u8) -> Res<Self> {
        Self::from_ipv4(Ipv4Addr::new(a, b, c, d))
    }

    pub fn from_ipv4(addr: Ipv4Addr) -> Res<Self> {
        use nix::sys::socket::{AddressFamily, SockFlag, SockType};
        let fd = nix::sys::socket::socket(AddressFamily::Inet, SockType::Datagram, SockFlag::empty(), None)?;

        let read_buf = [0u8; S];
        Ok(Self {
            read_buf,
            addr,
            socket: fd,
        })
    }

    pub fn from_octets([a, b, c, d]: [u8; 4]) -> Res<Self> {
        Self::new(a, b, c, d)
    }

    pub fn set_send_timeout(&mut self, timeout: Duration) -> Res<()> {
        let t = nix::sys::time::TimeVal::new(timeout.as_secs() as _, timeout.subsec_micros() as _);
        nix::sys::socket::setsockopt(&self.socket, nix::sys::socket::sockopt::SendTimeout, &t)
    }

    pub fn set_read_timeout(&mut self, timeout: Duration) -> Res<()> {
        let t = nix::sys::time::TimeVal::new(timeout.as_secs() as _, timeout.subsec_micros() as _);
        nix::sys::socket::setsockopt(&self.socket, nix::sys::socket::sockopt::ReceiveTimeout, &t)
    }

    pub fn sockaddr(&self, port: u16) -> SockaddrIn {
        let [a, b, c, d] = self.addr.octets();
        SockaddrIn::new(a, b, c, d, port)
    }

    pub fn send_raw(&self, bytes: &[u8], port: u16) -> Res<usize> {
        let wrote = nix::sys::socket::sendto(self.as_raw_fd(), bytes, &self.sockaddr(port), MsgFlags::empty())?;
        Ok(wrote)
    }

    pub fn recv_raw<'a>(&'a mut self) -> Res<&'a [u8]> {
        let read = nix::sys::socket::recv(self.as_raw_fd(), &mut self.read_buf, MsgFlags::empty())?;

        let ptr = self.read_buf.as_ptr();
        let data = unsafe { core::slice::from_raw_parts(ptr, read) };

        Ok(data)
    }

    pub fn recv_from_raw<'a>(&'a mut self) -> Res<(SockaddrIn, &'a [u8])> {
        let (read, addr) = nix::sys::socket::recvfrom::<SockaddrIn>(self.as_raw_fd(), &mut self.read_buf)?;
        let addr = addr.unwrap();

        let ptr = self.read_buf.as_ptr();
        let data = unsafe { core::slice::from_raw_parts(ptr, read) };

        Ok((addr, data))
    }
}

impl <const S: usize> AsRawFd for Socket<S> {
    fn as_raw_fd(&self) -> RawFd {
        self.socket.as_raw_fd()
    }
}
