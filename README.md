## BMP280

A simple library for using the Bosch BMP280 barometer and altimeter.

```rust
extern crate bmp280;
extern crate i2cdev;

use bmp280::Bmp280Builder;

fn main() {
    let mut dev = Bmp280Builder::new()
        .path("/dev/i2c-1".to_string())
        .address(0x77)
        .build()
        .expect("Failed to build device");

    dev.zero().expect("Device failed to zero");

    loop {
        println!("{:?} kPa", dev.pressure_kpa().unwrap());
        println!("{:?} m", dev.altitude_m().unwrap());
        println!("{:?} c", dev.temperature_celsius().unwrap());
        std::thread::sleep(std::time::Duration::from_millis(250));
    }

}
```
