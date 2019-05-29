BH1750-ehal
===========

Driver fro  BH1750 is an digital Ambient Light Sensor IC for I2C bus interface.

    let mut i2c = I2c::i2c3(p.I2C3, (scl, sda), 400.khz(), clocks);
    let mut delay = Delay::new(cmp.SYST, clocks);
    let mut bh1750 = BH1750::new(i2c, delay, Address::ADDR_L).unwrap();
    let light = hb1750.light_one_shot(OneTimeMeasurement::LOW_RES);
    let light = bh1750.get_measurement(ContinuesMeasurement::LOW_RES);
    println!("light {:.1} \n", light);
