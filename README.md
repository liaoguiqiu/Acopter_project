# Acopter_project

rotorcraft version 2.0

-This is a C/C++ autopolit project running on stm32 F407 (The UcosII is transfered to STM32) for:

1 Reading data and do filter for Accelerometers,Gyros, Magnets, barometers,GPS,Ultrasound

2 Fusing these data to estimate the pitch,roll,yaw,angular rate,position, velocity and acceleration of UAV

3 Controller

-The configration of the micro-processor is shown as below:

<p align="center">
    <img  src= "https://github.com/liaoguiqiu/Acopter_project/blob/master/config%20of%20stm32.jpg" >
</p>


-Please see the detail of PCB design in:
circuit diagram of mother board.pdf


ps: the stabliest navigation code is thie version of EKF at: Acopter_project/Acopter/Src/poliot/imu_ekf2.cpp

Contact: liaoguiqiu@outlook.com
