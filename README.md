# 2338 FRC 2023
![logo](https://avatars.githubusercontent.com/u/8992546?s=200&v=4)
#### Code for Gear It Forward's 2023 robot, [Zephyr](https://www.youtube.com/watch?v=i8rPwPnoQRg&pp=ygUIMjMzOCBmcmM%3D).
#### General Information:
* Written in Java
* Timed Robot Format
* VendorDeps: 
  * PhoenixProAnd5
  * REVLib
  * WPILibNewCommands
  * PathPlanner
* If you have any questions, feel free to contact us at
  * [Team Email](mailto:2338@sd308.org)
  * [Lead Programmer Email](mailto:17001055@sd308.org)
  * [GitHub](https://github.com/Team2338) - add questions as issues or whatever else you need
****
## Sections
1. [Collector](#collector)
   * Motor System
   * Pneumatic System
2. [Arm](#arm)
   * Angle Control
   * Set Positions
3. [Telescoping Arm](#telearm)
    * Arm telescoping
    * Set Positions
4. [Elevator](#elevator)
   * Elevator control
   * Set Positions
5. [Swerve Drivetrain](#swerve)
   * MK4 Swerve Modules
   * Swerve Drivetrain Control
   * Drive Modes
6. [LED Subsystem](#led)
   * LED Control and Management
7. [Vision](#vision)
   * Vision Processing
   * Vision Commands
8. [Spoiler](#spoiler)
   * Spoiler?
     * Spoiler!

******
Collector<a name="collector"></a>
----
Our collector is made up of two elements - a pneumatic system to set the shape of the game piece carried and a 
set of wheels to quickly intake said game piece.<br />
```java
public class CollectorWheels {
    private static final DoubleSolenoid solenoid = new DoubleSolenoid( PneumaticsModuleType.REVPH, RobotMap.SOLENOID_COLLECTOR_FORWARD, RobotMap.SOLENOID_COLLECTOR_REVERSE);
    private DoubleSolenoid.Value state = DoubleSolenoid.Value.kForward;
    
    public void wheelsOut() {
            state = DoubleSolenoid.Value.kReverse;
            Robot.ledSubsystem.LEDWheelsOut();
    }

    public void wheelsIn() {
        state = DoubleSolenoid.Value.kForward;
        Robot.ledSubsystem.LEDWheelsIn();
    }
}
```
Primary Subsystems and Commands to reference: <br />
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
collector eject speed. It also allows us to quickly store the arm away whenever on the go. 
In other words, it's our primary wrist of motion.<br />
```java
public class Arm extends SubsystemBase {
    public void move(double percent) {
        if (Robot.oi.aux.getHID().getRightStickButton()) {
            armMotor.configReverseSoftLimitThreshold(0);
        } else {
            armMotor.configReverseSoftLimitThreshold(Constants.Arm.MIN_POS);
        }

        // soft limits will keep the robot arm in allowable range
        armMotor.set(percent);
    }

    public void PIDMove() {
        armMotor.set(ControlMode.Position, armTargetPos);
    }
}
```
Primary Subsystems and Commands to reference: <br />
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
```java
public class TelescopingArm extends SubsystemBase {
    public void setMotorSpeed(double percent) {
        // do not allow for arm to go past soft limits
        if ( (percent > 0 && telescopingMotor.getEncoder().getPosition() > Constants.TelescopingArm.MAX_POS ) ||
                (percent < 0 && telescopingMotor.getEncoder().getPosition() < Constants.TelescopingArm.MIN_POS ) )
            percent = 0;

        telescopingMotor.set(percent);
    }

    public double getPosition() {
        return telescopingMotor.getEncoder().getPosition();
    }
}
```
Primary Subsystems and Commands to reference:<br />
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
double human player substations, primary movement method shown.<br />
```java
public class Elevator extends SubsystemBase {
    public void move(double percent) {
        elevatorMotor.set(ControlMode.PercentOutput, percent);
    }

    public void PIDHold() {
        elevatorMotor.selectProfileSlot(1,0);
        // the elevator needs a different kF when it is lower to the ground, otherwise it doesn't stay at the position
        if( elevatorTargetPos < Constants.Elevator.PLACE_CUBE_MID_POS) {
            elevatorMotor.config_kF(1, Constants.Elevator.F_HOLD_LOW);
        }
        else
            elevatorMotor.config_kF(1, Constants.Elevator.F_HOLD);

        elevatorMotor.set(ControlMode.Position, elevatorTargetPos); // closed loop position control
    }
}
```
Primary Subsystems and Commands to reference:<br />
* [Elevator.java](src/main/java/team/gif/robot/subsystems/Elevator.java)
* [ElevatorManualControl.java](src/main/java/team/gif/robot/commands/elevator/ElevatorManualControl.java)
* [ElevatorPIDControl.java](src/main/java/team/gif/robot/commands/elevator/ElevatorPIDControl.java)
* [SetElevatorPosition.java](src/main/java/team/gif/robot/commands/elevator/SetElevatorPosition.java)
***
Swerve Drivetrain<a name="swerve"></a>
---
This year, we decided on using a swerve chassis to take advantage of its ability to strafe in the 
community, as well as to get around defence in the center of the field as fast as possible.
We use SDS MK4's as our chosen swerve modules, configured with NEOs to turn, Falcons to drive, and
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
In addition to what a normal Swerve Drive would entail, we also added OTF (on-the-fly) drive modes as "strats" that
our drivers could use to get around various situations (e.g. defence, opponent robots, etc.). We structured these as an enum so that
we could have a single selection of a preset list.<br />
```java
public enum drivePace {
    COAST_FR(Constants.Drivetrain.COAST_SPEED_METERS_PER_SECOND, true),
    COAST_RR(Constants.Drivetrain.COAST_SPEED_METERS_PER_SECOND, false),
    SLOW_FR(Constants.Drivetrain.SLOW_SPEED_METERS_PER_SECOND, true),
    SLOW_RR(Constants.Drivetrain.SLOW_SPEED_METERS_PER_SECOND, false),
    BOOST_FR(Constants.Drivetrain.BOOST_SPEED_METERS_PER_SECOND, true),
    BOOST_RR(Constants.Drivetrain.BOOST_SPEED_METERS_PER_SECOND, false);

    private double value;
    private boolean isFieldRelative;

    drivePace(double value, boolean isFieldRelative) {
        this.value = value;
        this.isFieldRelative = isFieldRelative;
    }

    public double getValue() {
        return this.value;
    }
    public boolean getIsFieldRelative() {
        return this.isFieldRelative;
    }
}
```
For path generation during autonomous, we found out pretty early that the included TrajectoryGenerator class cannot 
generate holonomic paths. Our solution was [Team 3015 Ranger Robotics' fantastic PathPlanner](https://github.com/mjansen4857/pathplanner) application, allowing us to both
rotate and translate at the same time, just like in tele-op with field-relative instruction. <br />
```java
public class RobotTrajectory {
    public PPSwerveControllerCommand baseSwerveCommand(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
            trajectory,
            Robot.swervetrain::getPose,
            Constants.Drivetrain.DRIVE_KINEMATICS,
            new PIDController(Constants.AutoConstants.PX_CONTROLLER, 0, 0),
            new PIDController(Constants.AutoConstants.PY_CONTROLLER, 0, 0),
            new PIDController(Constants.AutoConstants.P_THETA_CONTROLLER, 0, 0),
            Robot.swervetrain::setModuleStates,
            Robot.swervetrain
        );
    }
}
```
Primary Subsystems and Commands to reference:<br />
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
the LEDs flash green to confirm collection. We set the value as a subsystem member,
then periodically call a method to set the member variables to the actual LEDs.<br />
```java
public class LEDSubsystem extends SubsystemBase {
    public void setColors() {
        for (int i = 0; i < RobotMap.HP_LEDS.length; i++) {
            ledBuffer.setRGB(RobotMap.HP_LEDS[i],HP[0],HP[1], HP[2]);
            led.setData(ledBuffer);
        }
        for (int j = 0; j < RobotMap.GAME_PIECE_LEDS.length; j++) {
            ledBuffer.setRGB(RobotMap.GAME_PIECE_LEDS[j], GamePiece[0], GamePiece[1], GamePiece[2]);
            led.setData(ledBuffer);
        }
        for (int k = 0; k < RobotMap.WHEEL_STATE_LEDS.length; k++) {
            ledBuffer.setRGB(RobotMap.WHEEL_STATE_LEDS[k], WheelState[0], WheelState[1], WheelState[2]);
            led.setData(ledBuffer);
        }
    }

    public void setLEDHPColor(int r, int g, int b) {
        HP[0] = r;
        HP[1] = g;
        HP[2] = b;
    }
}
```
Primary Subsystems and Commands to reference: <br />
* [LEDSubsystem.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fsubsystems%2FLEDSubsystem.java)
* [LEDSubsystemDefault.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fled%2FLEDSubsystemDefault.java)
* [CubeLED.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fled%2FCubeLED.java)
* [ConeLED.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fled%2FConeLED.java)
***
Vision<a name="vision"></a>
---
We've been using the Limelight for a few years now, and this year was no different, except for one thing - YOLO.
YOLO (You Only Look Once) is a neural network algorithm that can detect objects in real time, and we've been using it to detect and navigate to game pieces
from the double substation. To support YOLO, we need previous data, as well as, Neural Network acceleration to process.
Limelight kindly provides us with the previous data, and to support acceleration, we've used a Google Coral. Shown is navigating to auto collect a game piece.<br />
```java
public class LimeLightAutoCollect extends CommandBase {
    @Override
    public void execute() {
        double rotationSpeed = 0;
        if (count < 20) {
            if (Robot.limelightHigh.hasTarget()) {
                xOffset = Robot.limelightHigh.getXOffset();
                rotationSpeed = (Math.abs(xOffset) < yTolerence) ? 0 : ((xOffset > 0) ? -0.4 : 0.4);
                if (rotationSpeed == 0) {
                    count++;
                } else {
                    count = 0;
                }
            }

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,rotationSpeed); //turns towards xoffset and drives forward
            SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
            Robot.swervetrain.setModuleStates(moduleStates);
        }

        if (Robot.elevator.getPosition() > (Constants.Elevator.LOAD_FROM_DOUBLE_SUBSTATION_POS - 3 * Constants.Elevator.EL_TICKS_PER_INCH) &&
                Robot.arm.getPosition() > (Constants.Arm.LOAD_FROM_DOUBLE_SUBSTATION_POS - 5 * Constants.Arm.TICKS_PER_DEGREE) &&
                Robot.arm.getPosition() < (Constants.Arm.LOAD_FROM_DOUBLE_SUBSTATION_POS + 3 * Constants.Arm.TICKS_PER_DEGREE)) {


            Robot.ledSubsystem.clearLEDGamePieceColor();

            if (count == 20 && Robot.telescopingArm.getPosition() < Constants.TelescopingArm.MAX_POS) {
                Robot.telescopingArm.setMotorSpeed(Constants.TelescopingArm.HIGH_VELOCITY);
            } else {
                Robot.telescopingArm.setMotorSpeed(0);
            }
        } else {
            Robot.ledSubsystem.setLEDAutoAlignError();
        }
    }
}
```
We can also auto align to the cone nodes using a retroreflective limelight pipeline, 
referenced in [LimeLightAutoAlign.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fautoaim%2FLimeLightAutoAlign.java) by moving left or right
at a constant velocity until the tx is about 0.<br />
Primary Subsystems and Commands to reference: <br />
* [Limelight.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fsubsystems%2Fdrivers%2FLimelight.java)
* [LimeLightAutoCollect.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fautoaim%2FLimeLightAutoCollect.java)
* [LimeLightAutoAlign.java](src%2Fmain%2Fjava%2Fteam%2Fgif%2Frobot%2Fcommands%2Fautoaim%2FLimeLightAutoAlign.java)
***
Spoiler<a name="spoiler"></a>
---
**Developer comment: Spoiler**<br />
Due to the limited velocity of our swervy boi relative to other bots,
we've decided to add a spoiler to our robot to increase our top speed by adding an approximate
25 tonnes of downforce on the straight. It also helps us to easily get up and balance during endgame.