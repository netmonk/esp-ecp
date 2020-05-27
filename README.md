# esp-ecp

## Explanations

This project is first attempt to user esp32 mcu as a flight controller for quadcopter.

The first step is to get engine running. 

Currently the code is sending dshot600 frame to an esc usint RMT functions from esp32-HAL. 

The current setup is based on a potentiometer plugged on the pin d34, and the main loop read the value to compute a throttle command to send to the esc. 

The esc input is plugged on pin 5 of the devkit, and the ground of the esc signal is also attached to the ground pin of the devkit. 

You can find a video of the exemple here: [https://streamable.com/dxvt4n](https://streamable.com/dxvt4n)

More successfull video here : [https://streamable.com/aua209](https://streamable.com/aua209)

and with a blheli_32 esc from aikon and more recent emax 2205 2300kv motor : [https://streamable.com/c3l6em](https://streamable.com/c3l6em)


## Hardware setup 

* esp32 devkitv1 esp-wroom-32
* esc racerstar rsv2 (blheli_s)
* motor is racerstar 2205 at 2400kv
* power board distribution matek system 4*25amp.
* potentiometer
* a breadboard and few cables :)

## Shoutouts

This code was first based on : 
[JyeSmith dshot-esc-test](https://github.com/JyeSmith/dshot-esc-tester)
Big shout outs to [@JyeSmith](https://github.com/JyeSmith)