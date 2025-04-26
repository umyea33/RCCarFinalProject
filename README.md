# RCCarFinalProject
Authors:
    Luke Jones
    John Cimmarusti
    Jaiden Kazemini
    Elden Harrison

Gyro Controlled RC Car:
    The goal of this project is to make a fun little RC car that is controlled with the movement of your hand via a gyroscope.  Once both things are powered on, the car will move based on the rotational position of the STM (typically in your hand).  There are 4 commands Left, Right, Forward, and Backward each corresponding to one direction of rotation on the gyroscope.  These commands are based on the movement relative to the gyroscopes starting position.  If it is on and you do not like the default postion, holding down the reset button and moving it to a desired default position works well.

Setup:
    The transmitter portion has two parts, a gyroscope and transceiver (transmitter).
    The receiver portion also has two main parts, a transceiver (receiver) and 2 motor drivers connected to motors.

    Gyro:
        The gyroscope used is the one on the STM32F072 and additional wiring is required to make it work.  This is shown in Gyroscope Wiring.png located in the same file as this readME.

    Transceiver:
        The receiver and transmitter are set up the exact same way on two different STMs.  The transciever has 8 pins and they are maped to pins as follows: MISO (PB4), IRQ (PB8), SCK (PB3), MOSI (PB5), CE (PB7), CSN (PB6), VCC (3.3V), and Ground (Gnd).

    Motor Driver:
        The motor driver needs to be connected to the power source, and also has 6 pins that connect to pins on the STM.  These pins are as follows ENA (PC10), In1 (PC8), In2 (PC9), In3 (PC3), In4 (PC4), and ENB (PC11).

    With these 3 things setup right (4 Counting the transmitter and receiver separate), the car will work as described above.

