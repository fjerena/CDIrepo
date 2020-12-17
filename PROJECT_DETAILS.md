# Project Details

Firstly, I decided to project an Electronic Ignition System (DC CDI) because I consider one of the most significant aspects to increase substantially the 
engine torque and power, responsible to optmize ignition timing and inclusive increase maximum engine speed. My intention is share my personal knowledge 
and experience in this development area, where I had good results with related projects, I will present you how to create a simple circuit can be build by 
people with limited resources, but big curiosity and desire to learn and experiment and achieve good results changing old ignition systems for a new one 
more efficient...

## Core module 
To create this project I choose the Blue Pill dev board because:
- Low cost and easy to found it
- Hardware is powerful, available different communication modules like: I2C, SPI, CAN, USART and another important resourses like timers, AD converters etc
- Development tool to debug and trace software: flash, download, breakpoint debugging, register and memory view, serial wire trace
- Small size and very good design to create external shields using perforated boards (fast prototype)

PS: For more detailed information consult [Datasheet STM32F103C8](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)

## Power Supply module

I decide to use a Step-Down Voltage Regulator to optimize the power consumption (increase battery autonomy)

## Capacitor Discharger Ignition  

An important choose was include an external inverter board, this circuit belongs to DC CDI archteture, responsible to generate a high-voltage (300Vp approximately)
and enable the capacitor discharging provide more than 20mJ energy, is enough for generate a good spark for naturally aspirated engine. To understand how to 
control ignition timing electronically and the advantages to do it, read this [article]https://en.wikipedia.org/wiki/Ignition_timing).

In resume, this module will receive a crankshaft position signal and generate a trigger to fire the spark during adequate crankshaft angular position to optmize 
torque generation after fuel combustion!

## Bill of materials:

- [Step-Down Power Supply module](https://www.aliexpress.com/item/32970694265.html?spm=a2g0o.productlist.0.0.5fed36e219TvMn&algo_pvid=d335ded9-f9dc-4fb1-a5ef-1f18dc80345c&algo_expid=d335ded9-f9dc-4fb1-a5ef-1f18dc80345c-53&btsid=0be3764515900535016104332e80e6&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_)
- [Blue Pill](https://www.aliexpress.com/item/32839140960.html?spm=a2g0o.productlist.0.0.623b73477OAII5&algo_pvid=3eb4afd7-9750-40da-b9bb-a72602a37bc8&algo_expid=3eb4afd7-9750-40da-b9bb-a72602a37bc8-12&btsid=0ab6f82415898199543501406e6c04&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_)
- [Inverter 12V DC to 220V AC - 40W](https://www.aliexpress.com/item/32929609179.html?spm=a2g0o.productlist.0.0.25aa20c2hvNnvt&algo_pvid=4aa6c4d7-b56d-4c59-b328-d9101e6d2580&algo_expid=4aa6c4d7-b56d-4c59-b328-d9101e6d2580-28&btsid=0ab6f83a15900527095032060e068e&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_)
- [OLED 0,96" Display](https://www.aliexpress.com/item/32667396994.html?spm=a2g0o.productlist.0.0.627530e1WnnvQE&algo_pvid=19297b5b-2f8c-4695-9e26-bda77824da58&algo_expid=19297b5b-2f8c-4695-9e26-bda77824da58-18&btsid=0be3764315900530466002365ec396&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_)
- Perforated board (CDI circuit)

## Software approach

To implement the software control for this module I will use a Real Time Operational System to schedulle tasks that need performed and implement a software oriented
by hardware interrupt to ensure accuracy and cadence.
During the project development, I will update the newest solutions and algorithms adopted!

### Kalman filtering
- https://en.wikipedia.org/wiki/Kalman_filter
- http://www.megamanual.com/ms2/configure.htm

### Flywheel sensor device:
- 18 degree BTDC - Idle          (define pulse edge - rising)
- 40 degree BTDC - Max advance   (define pulse edge - falling)

### Reference
[How system can measure the crankshaft sensor position](http://www.sportdevices.com/ignition/ignition.htm)

