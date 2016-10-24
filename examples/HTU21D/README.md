HTU21D with Brzo I2C
===================
For the tests I used [Adafruit's HTU21 Breakout Board](https://www.adafruit.com/product/1899). The brzo i2c library is working with the board, both with or without clock stretching. However, it seems that this breakout board has some issues, when it is used with an esp8266 and 3.3V. T

Breakout Board Issues
----------
The breakout board supports 5V or 3.3V and uses MOSFETs (BSS138) to do the level shifting, besides this, high value pullups (10kOhm) are placed on both SDA and SCL. This combinations leads to a raise time which far too long. 
With my measurements Vdd was 3.169V, 70% was 2.232V and 30% 0.9567V. According to the i2c specification, the raise time should be maximum 300 nsec, however in my measuremts it was around 800 nsec. You can get better raise times when adding "additional" pull ups, e.g., 1.5kOhm. However this will *not* solve the issue that the device is not responding from time to time.

![enter image description here](https://drive.google.com/uc?export=view&id=0B4TuuOHSvMcKMzRYa1NYWUpzLWs)

As a consequence, sometimes the breakout board (i.e. the HTU21) is not responding—which of course leads to brzo i2c error 2. 

Sometimes, there is simply no response form the HTU21 at address 0x40. 

![enter image description here](https://drive.google.com/uc?export=view&id=0B4TuuOHSvMcKaENzYnBpRGNiaTQ)

![enter image description here](https://drive.google.com/uc?export=view&id=0B4TuuOHSvMcKaW10MmVOcmlIQ2s)


**When does the NACK occur? I.e. the HTU21 does not respond**


Sometimes:
At startup (i.e. `setup`) when Soft Reset Commands are sent. Since at least one Soft Reset is important, the example code tries 3 times. A typcial behaviour is shown as serial output:

    Port open
    Soft reset fail. Brzo error : 2
    Soft reset OK 

Periodacally: 
After a successful temperature measurement, there is always a NACK on address 0x40 as the serial output shows:

    Waiting 5 seconds...
    SCL Stretch duration in usec: 42430
    Temp = 26.04848862
    Waiting 5 seconds...
    SCL Stretch duration in usec: 0
    Brzo error : 2
    Waiting 5 seconds...
    SCL Stretch duration in usec: 42430
    Temp = 26.06993866
