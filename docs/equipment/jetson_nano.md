# The Jetson Nano

![Nano](https://s3.amazonaws.com/cms.ipressroom.com/219/files/20192/jetson-nano-family-press-image-hd.jpg){width=500}

Some useful links:

- [Jetson Nano Datasheet](https://components101.com/sites/default/files/component_datasheet/Jetson-Nano-DataSheet.pdf)

### Using the Pins on the Jetson

Here's an article with the pinout of the Jetson.

- [Jetson Nano pinout](https://jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/)

If you're using python, you're most likely going to be interfacing with the Jetson using the `RPi` library. Here's the documentation for the python `RPi.GPIO` module.

- [RPi.GPIO documentation](https://sourceforge.net/p/raspberry-gpio-python/wiki/Home/)
- [PWM Control Tutorial through RPi.GPIO](https://digitalshack.org/raspberry-pi-pwm-tutorial/)

It's also possible to interface with the GPIO pins through the filesystem as linux exposes them as a virtual file system.

- [A c++ GPIO guide that uses the filesystem](https://jetsonhacks.com/2015/12/29/gpio-interfacing-nvidia-jetson-tx1/), [the associated github repo](https://github.com/jetsonhacks/jetsonTX1GPIO).
- [A quick overview of the GPIO filesystem](https://www.electronicsforu.com/electronics-projects/hardware-diy/accessing-gpios-using-sysfs-interface-linux)