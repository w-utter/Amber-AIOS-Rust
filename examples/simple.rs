
use amber_aios::*;
use amber_aios::socket::Socket;
use serde_json::json;
use std::time::Duration;

fn main() {
    let mut sock = Socket::<2048>::new(192, 168, 1, 31).unwrap();
    sock.set_read_timeout(Duration::from_millis(2000)).unwrap();

    let mut motor = AiosMotor::<2048>::from_socket(sock);

    motor.enable::<1>();

    loop {
        match motor.get_cvp() {
            Ok(cvp) => {
                println!("{cvp:?}");
            }
            Err(e) => {
                println!("err: {e:?}");
            }
        }

        if let Ok(cvp) = motor.get_cvp() {
        }
        println!("looping")
    }

    /*
    let err = motor.err::<0>().unwrap();

    //println!("getting root");
    let root = motor.get_root().unwrap();
    let _ = motor.enable::<1>().unwrap();
        //let _ = sock.enable::<0>();

    let set_cmd = json!({
        "method": "SET",
        "reqTarget": "/IO_State",
        //"reply_enable": true,
    });

    let get_cmd = json!({
        "method": "GET",
        "reqTarget": "/IO_State",
        //"reply_enable": true,
    });

    if let Ok(a) = motor.send_recv(&get_cmd, 2333) {
        let str = unsafe { std::str::from_utf8_unchecked(a) };
        println!("{str:?}");
    }
    */
}
