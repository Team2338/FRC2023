# 2338 FRC 2023
![logo](https://avatars.githubusercontent.com/u/8992546?s=200&v=4)
#### Code for Gear It Forward's 2023 robot, [Przesada](https://www.youtube.com/watch?v=i8rPwPnoQRg&pp=ygUIMjMzOCBmcmM%3D).
#### General Notes:
* Written in Java
* Timed Robot Format
* VendorDeps: 
  * PhoenixProAnd5
  * REVLib
  * WPILibNewCommands
****
## Sections
1. [Collector](#collector-a-namecollector-a)
   * Motor System
   * Pneumatic System
2. [Arm](#arm-a-namearm-a)
   * Angle Control
   * Set Positions
3. [Telescoping Arm](#telescoping-arm-a-nametelearm-a)
    * Arm telescoping
    * Set Positions
4. [Elevator](#elevator-a-nameelevator-a)
   * Elevator control
   * Set Positions
5. [Swerve Drivetrain](#swerve-drivetrain-a-nameswerve-a)
   * MK4 Swerve Modules
   * Swerve Drivetrain Control
   * Drive Modes
6. [LED Subsystem](#led-subsystem-a-nameled-a)
   * LED Control and Management
7. [Drivers & Libraries](#drivers--libraries-a-namedrivers-a)
   * Limelight
   * Pigeon
   * Compressor
   * Logging
8. [Autos & Routines](#autos--routines-a-nameautos-a)
   * Time-Based Autos
   * Combo Commands

******
Collector<a name="collector"></a>
----
Our collector is made up of two elements - a pneumatic system to set the shape of the game piece carried and a 
set of wheels to quickly intake said game piece.<br />
Subsystems and Commands to reference: <br />
* [Collector.java](src/main/java/team/gif/robot/subsystems/Collector.java)<br />
* [CollectorWheels.java](src/main/java/team/gif/robot/subsystems/CollectorWheels.java)<br />
* [WheelsIn.java](src/main/java/team/gif/robot/commands/collector/WheelsIn.java)<br />
* [WheelsOut.java](src/main/java/team/gif/robot/commands/collector/WheelsOut.java)<br />
* [CollectorEject.java](src/main/java/team/gif/robot/commands/collector/CollectorEject.java)<br />
* [CollectorCollect.java](src/main/java/team/gif/robot/commands/collector/CollectorCollect.java)<br />
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
Our telescoping arm is a subsystem controlled by a single motor, used to control the extension
of the arm. Whenever we need to place high, it requires a longer distance to be covered, both vertically 
and horizontally - the telescoping arm takes care of the horizontal extension, while the vertical 
can be taken care of by the angle of the arm and the elevator height.<br />
Subsystems and Commands to reference:<br />
* [TelescopingArm.java](src/main/java/team/gif/robot/subsystems/TelescopingArm.java)
* [ArmIn.java](src/main/java/team/gif/robot/commands/telescopingArm/ArmIn.java)
* [ArmMid.java](src/main/java/team/gif/robot/commands/telescopingArm/ArmMid.java)
* [ArmOut.java](src/main/java/team/gif/robot/commands/telescopingArm/ArmOut.java)
* [MoveArm.java](src/main/java/team/gif/robot/commands/telescopingArm/MoveArm.java)
***
Elevator<a name="elevator"></a>
---
Our elevator is a subsystem controlled by a single motor, used to lift the arm and the
telescoping arm for placement at any level, as well as collection from both the single and
double human player substations.<br />
Subsystems and Commands to reference:<br />
* [Elevator.java](src/main/java/team/gif/robot/subsystems/Elevator.java)
* [ElevatorManualControl.java](src/main/java/team/gif/robot/commands/elevator/ElevatorManualControl.java)
* [ElevatorPIDControl.java](src/main/java/team/gif/robot/commands/elevator/ElevatorPIDControl.java)
* [SetElevatorPosition.java](src/main/java/team/gif/robot/commands/elevator/SetElevatorPosition.java)
***
Swerve Drivetrain<a name="swerve"></a>
---
This year, we decided on using a swerve chassis to take advantage of its ability to strafe in the 
community, as well as to get around defence in the center of the field as fast as possible.
We use SMS MK4's as our chosen swerve modules, configured with NEOs to turn, Falcons to drive, and
CANCoders to track the direction of the wheel. Unlike other examples you may encounter, we opted not to use WPILibs tools 
to optimize the state, as we found it was far too slow and inaccurate on our bot.
Instead, we found the fastest way to get to any given state on a disconnected 0->360 degree axis.
```java
public class SwerveModuleMK4 {
   /**
    * Optimize the swerve module state by setting it to the closest equivalent vector
    * @param original the original swerve module state
    * @return the optimized swerve module state
    */
   private SwerveModuleState optimizeState(SwerveModuleState original) {
      // Compute all options for a setpoint
      double position = getTurningHeading();
      double setpoint = original.angle.getRadians();
      double forward = setpoint + (2 * Math.PI);
      double reverse = setpoint - (2 * Math.PI);
      double antisetpoint = findRevAngle(setpoint);
      double antiforward = antisetpoint + (2 * Math.PI);
      double antireverse = antisetpoint - (2 * Math.PI);

      // Find setpoint option with minimum distance
      double[] alternatives = { forward, reverse, antisetpoint, antiforward, antireverse };
      double min = setpoint;
      double minDistance = getDistance(setpoint, position);
      int minIndex = -1;
      for (int i = 0; i < alternatives.length; i++) {
         double dist = getDistance(alternatives[i], position);
         if (dist < minDistance) {
            min = alternatives[i];
            minDistance = dist;
            minIndex = i;
         }
      }

      // Figure out the speed. Anti- directions should be negative.
      double speed = original.speedMetersPerSecond;
      if (minIndex > 1) {
         speed *= -1;
      }

      return new SwerveModuleState(speed, new Rotation2d(min));
   }
}
```
We decoupled the SwerveModule class from the SwerveDrivetrain class due to the base SwerveModule code's
usability across multiple module types and motors.<br />
Subsystems and Commands to reference:<br />
* [SwerveModuleMK4.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fsubsystems%2Fdrivers%2FSwerveModuleMK4.java)
* [SwerveDrivetrain.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fsubsystems%2FSwerveDrivetrain.java)
* [drivePace.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Flib%2FdrivePace.java)
* [EnableBoost.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2FdriveModes%2FEnableBoost.java)
* [DriveSwerve.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fdrivetrain%2FDriveSwerve.java)
* [ResetHeading.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fdrivetrain%2FResetHeading.java)
* [ResetWheels.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fdrivetrain%2FResetWheels.java)
***
LED Subsystem<a name="led"></a>
---
For the first time in a long time, we've decided to put LEDs on our robot.
Primarily used for alerting the human player of what game piece the driver would like, 
whether purple or yellow and vice versa, whenever the driver picks up a game piece, 
the LEDs flash green to confirm collection.<br />
Subsystems and Commands to reference: <br />
* [LEDSubsystem.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fsubsystems%2FLEDSubsystem.java)
* [LEDSubsystemDefault.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fled%2FLEDSubsystemDefault.java)
* [CubeLED.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fled%2FCubeLED.java)
* [ConeLED.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fled%2FConeLED.java)
***
Drivers & Libraries<a name="drivers"></a>
---

***
Autos & Routines<a name="autos"></a>
---

***