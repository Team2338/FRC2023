# FRC2023
#### Code for Gear It Forward's 2023 robot, [Przesada](https://www.youtube.com/watch?v=i8rPwPnoQRg&pp=ygUIMjMzOCBmcmM%3D). 
****
## Sections
1. [Collector](#collector-a-namecollector-a)
   1. Motor System
   2. Pneumatic System
2. [Arm](#arm-a-namearm-a)
   1. Angle Control
   2. Set Positions
3. [Telescoping Arm](#telescoping-arm-a-nametelearm-a)
    1. Arm telescoping
    2. Set Positions
4. [Elevator](#elevator-a-nameelevator-a)
   1. Elevator control
   2. Set Positions
5. [Swerve Drivetrain](#swerve-drivetrain-a-nameswerve-a)
   1. MK4 Swerve Modules
   2. Swerve Drivetrain Control
   3. Drive Modes
6. [LED Subsystem](#led-subsystem-a-nameled-a)
   1. LED Control and Management
7. [Drivers](#drivers-a-namedrivers-a)
   1. Limelight
   2. Pigeon
   3. Compressor
8. [Autos](#autos-a-nameautos-a)
   1. Time-Based
   2. Path Planner Usage

******
Collector<a name="collector"></a>
----
Our collector is made up of two elements - a pneumatic system to set the shape of the game piece carried and a 
set of wheels to quickly intake said game piece.<br />
Subsystems and Commands to reference: <br />
* [Collector.java](src/main/java/team/gif/robot/subsystems/Collector.java) - contains all code for the collector wheels <br />
* [CollectorWheels.java](src/main/java/team/gif/robot/subsystems/CollectorWheels.java) - contains all code for the collector pneumatics <br />
* [WheelsIn.java](src/main/java/team/gif/robot/commands/collector/WheelsIn.java) - Pulls the collector wheels in using pneumatics <br />
* [WheelsOut.java](src/main/java/team/gif/robot/commands/collector/WheelsOut.java) - Pushes the collector wheels out using pneumatics <br />
* [CollectorEject.java](src/main/java/team/gif/robot/commands/collector/CollectorEject.java) - Spins the collector wheels out<br />
* [CollectorCollect.java](src/main/java/team/gif/robot/commands/collector/CollectorCollect.java) - Spins the collector wheels in<br />
***
Arm<a name="arm"></a>
---
Our arm is a subsystem controlled by a single motor, used to control the angle. It allows us 
to maximize the reach of the arm without having to compensate with telescoping distance and 
collector eject speed. It also allows us to quickly store the arm away whenever on the go.<br />
Subsystems and Commands to reference: <br />
* [Arm.java](src/main/java/team/gif/robot/subsystems/Arm.java)
* [ArmLift.java](src/main/java/team/gif/robot/commands/arm/ArmLift.java)
* [ArmManualControl.java](src/main/java/team/gif/robot/commands/arm/ArmManualControl.java)
* [ArmPIDControl.java](src/main/java/team/gif/robot/commands/arm/ArmPIDControl.java)
* [SetArmPosition.java](src/main/java/team/gif/robot/commands/arm/SetArmPosition.java)
***
Telescoping Arm<a name="telearm"></a>
---
***
Elevator<a name="elevator"></a>
---
***
Swerve Drivetrain<a name="swerve"></a>
---
***
LED Subsystem<a name="led"></a>
---
***
Drivers<a name="drivers"></a>
---
***
Autos<a name="autos"></a>
---
***