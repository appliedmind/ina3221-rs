extern crate ina3221;
extern crate linux_embedded_hal as hal;

use hal::I2cdev;
use ina3221::{INA3221, A0, Config, AveragingMode, OperationMode};

fn main() {
    let i2c_bus = I2cdev::new("/dev/i2c-1").unwrap();

    // Start continuous conversion
    let mut ina = INA3221::new_config(i2c_bus,
                                      // Device address
                                      A0::Gnd.address(),
                                      Config::default().average(AveragingMode::N64)).unwrap();

    for (ch, channel) in ina.read_all_channels().unwrap().iter().enumerate() {
        println!("ch{} voltage: {:?} current: {:?}", ch, channel.voltage(), channel.current(0.01f32));
    }

    let shunt_sum = ina.read_sum().unwrap();
    println!("shunt sum voltage: {:?}", shunt_sum);

    // Start a single conversion on ch1 only
    ina.write_config(Some(
        Config::default()
        .mode(OperationMode::ShuntBusSingle)
        .ch2_en(false)
        .ch3_en(false)
    )).unwrap();

    //Loop until conversion is done
    loop {
        if ina.read_cvrf().unwrap_or(false) {
            break;
        }
    }

    let channel = ina.read_channel(0).unwrap();
    println!("ch1 single-shot voltage: {:?} current: {:?}", channel.voltage(), channel.current(0.01f32));

}