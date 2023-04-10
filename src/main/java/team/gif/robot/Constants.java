// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Drivetrain {
        //public static final double DRIVE_WHEEL_RADIUS = 0.05; // meters? Must be unit of velocity

        public static final boolean kFrontLeftTurningEncoderReversed = false; //false
        public static final boolean kRearLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kRearRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kRearLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kRearRightDriveMotorReversed = false;

        public static final boolean kFrontLeftTurningMotorReversed = true;
        public static final boolean kRearLeftTurningMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = false;
        public static final boolean kRearRightTurningMotorReversed = false;

        public static final double FRONT_LEFT_OFFSET = 82.089;
        public static final double REAR_LEFT_OFFSET = -138.25195;
        public static final double FRONT_RIGHT_OFFSET = -20.3906;
        public static final double REAR_RIGHT_OFFSET = 157.85156;

        // Distance between centers of right and left wheels on robot
        public static final double TRACK_LENGTH = Units.inchesToMeters(25);

        // Distance between front and back wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.4375);

        // location of wheels from center of robot using following axis
        //        +x
        //         ^
        //         |
        //  +y  <---
        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2), // front left
                new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2), // front right
                new Translation2d(-TRACK_LENGTH / 2, TRACK_WIDTH / 2), // back left
                new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2)); // back right

        public static final boolean kGyroReversed = false;

        public static final double COAST_DRIVE_RPM = 2500; // 2750; //4800 demo speed //2750
        public static final double BOOST_DRIVE_RPM = 1675; // 1675 is max speed; was 1750;
        public static final double SLOW_DRIVE_RPM = 3500;

        public static final double COAST_SPEED_METERS_PER_SECOND = COAST_DRIVE_RPM *
            (Math.PI * Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
            (60.0 * Constants.ModuleConstants.GEAR_RATIO);

        public static final double BOOST_SPEED_METERS_PER_SECOND = BOOST_DRIVE_RPM *
                (Math.PI * Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
                (60.0 * Constants.ModuleConstants.GEAR_RATIO);

        public static final double SLOW_SPEED_METERS_PER_SECOND = SLOW_DRIVE_RPM *
                (Math.PI * Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
                (60.0 * Constants.ModuleConstants.GEAR_RATIO);
        public static double kMaxAccelerationMetersPerSecondSquared = 2;// TODO
    }

    public static final class ModuleConstants {
        public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 6 * (2 * Math.PI); //6
        public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 6 * (2 * Math.PI); //7
        public static final double GEAR_RATIO = 27.0 / 4.0; // need to ask aaron
        public static final double ENCODER_CPR = 2048.0;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.78);
        public static final double DRIVE_ENCODER_ROT_2_METER = Math.PI * WHEEL_DIAMETER_METERS / (GEAR_RATIO * ENCODER_CPR);
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
         //4096.0 for talons
        public static final double kDriveEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;

        public static final double kTurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) ENCODER_CPR;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 5;

        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;
        public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TURNING_MOTOR_GEAR_RATIO = 1.0 / 18.0;

        public static final double TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;

        public static final double TURNING_ENCODER_RPM_2_RAD_PER_SECOND = TURNING_ENCODER_ROT_TO_RAD / 60;

        public static final class DrivetrainPID {
            public static final double frontLeftP = 0.35; //pBot 0.4 all P
            public static final double frontLeftFF = 0.01; //pBot 0.01 all FF
            public static final double frontRightP = 0.35;
            public static final double frontRightFF = 0.01; //issa good
            public static final double rearLeftP = 0.35;
            public static final double rearLeftFF = 0.01;
            public static final double rearRightP = 0.35; // 0.6
            public static final double rearRightFF = 0.01;
        }
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 5.0;
        public static final double PY_CONTROLLER = 5.0;
        public static final double P_THETA_CONTROLLER = 3.7;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                    MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double DRIVE_SUPER_FAST = 1.0;
        public static final double DRIVE_FAST = 0.7;
        public static final double DRIVE_MEDIUM = 0.6;
        public static final double DRIVE_SLOW = 0.27; // 0.3;
        public static final double DRIVE_SUPER_SLOW = 0.2;
        public static final double HOLD_AT_ANGLE = 0.15;
        public static final double DRIVE_TIME_DEFAULT = 1.7; // seconds until the bot gets to the charging station
    }

    public static final class Joystick {
        public static final double DEADBAND = 0.1;
    }

    public static class Arm {
        public static final double FF = -0.04; //wasd -0.12; //was -0.08; // was -0.04; // -0.02 // arm is going down
        public static final double P = 1.5; // wasd 2.0; //was 1.0 // 2.0; // 2.0; // 1.2; // arm is going down
        public static final double REV_FF = FF; // -0.15; // wasd -0.04; //was -0.02; // -0.03 // arm is going up
        public static final double REV_P = 1.0; // 2.0; //was 2.0; // // arm is going up
        public static final double I_GT_45 = 0.003;
        public static final double I_LT_45 = 0.0;
        public static final double D = 0.0;
        public static final double Ticks_Move = 3;

        public static final double GRAV_FEED_FORWARD = 700 / 1023.0; // Percent constant to counteract gravity
        public static final double REV_GRAV_FEED_FORWARD = 700 / 1023.0;
        public static final double F = 16.0; // 0.8; // 0.4; // 0.3; // 0.425;
        public static final double REV_F = 16.0; // 0.3; // 0.38;

        // general motor function parameters
        public static final double PEAK_OUTPUT_FORWARD = 0.70; //was 0.55; // down // Percent motor command, arm is much faster than elevator
        public static final double PEAK_OUTPUT_FORWARD_CUBE_HIGH_POS = 0.3; //was 0.35;
        public static final double PEAK_OUTPUT_FORWARD_CONE_HIGH_POS = 0.25; // wasg 0.35;
        public static final double AUTO_OUTPUT_CONE_HIGH_POS = .25;
        public static final double AUTO_OUTPUT_CONE_HIGH_STAGE = .80;
        public static final double AUTO_OUTPUT_CONE_HIGH_FAST = .30;
        public static final double PEAK_OUTPUT_FORWARD_CUBE_MID_POS = 0.30;
        public static final double PEAK_OUTPUT_REVERSE = -0.80; //was -0.55 // arm is going up

        // motion magic parameters (not currently used)
//        public static final int MAX_VELOCITY = 16000; //16000 // 5000; // ticks/100ms
//        public static final int REV_MAX_VELOCITY = 16000;
//        public static final int MAX_ACCELERATION = 16000; // ticks/100ms/s

        public static final double TICKS_PER_DEGREE = 30.91;//was 31.378; // PBOT 26.8
        public static final double ZERO_OFFSET_TICKS = 230; //was 319; // was 303; // PBOT 375; // ticks between motor 0 and straight up (compass 0)
        public static final double PID_TOLERANCE = 3.0 * TICKS_PER_DEGREE; // allows arm to be within 3 degrees of target

        // n is in degrees
        // 90.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS; is 90 degrees, 0 straight up
        public static final double LOAD_FROM_DOUBLE_SUBSTATION_POS = 90.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS;
        public static final double LOAD_FROM_SINGLE_SUBSTATION_POS = 47.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS; //PbOT 45.0
        public static final double LOAD_FROM_GROUND_POS = 103.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS;
        public static final double PLACE_CUBE_HIGH_POS = 90.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS;
        public static final double PLACE_CUBE_HIGH_POS_AUTO = -45.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS;
        public static final double PLACE_CUBE_MID_POS = 68.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS; // was 105
        public static final double PLACE_CONE_HIGH_POS = 81.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS;
        public static final double PLACE_CONE_MID_POS = 90.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS;//90
        public static final double PLACE_CONE_MID_REAR_POS = -50 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS;
        public static final double PLACE_LOW_POS = 105.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS;

        public static final double STAGE_POS = 30.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS; // this is the location which is safe to go to/from home
//        public static final double MOVE_FROM_HOME_PRE_POS = LOAD_FROM_SINGLE_SUBSTATION_POS; // TODO is this necessary?
        public static final double HOME_POS = 25.0 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS; // PBOT 20

        public static final double ARM_80 = 80 * TICKS_PER_DEGREE + ZERO_OFFSET_TICKS;

        public static final double MAX_POS = PLACE_LOW_POS; // should be the largest of all positions above
        public static final double MIN_POS = HOME_POS; // should be the smallest of all positions above
    }

    public static class Elevator {
        // PID constants
        public static final double P = 1.5; // wasg 2.0; //was 4.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.1; // wasg 0.05; //was 0.8;
        public static final double REV_F = 0.1; //was 0.3;

        public static final double P_HOLD = 0.5; //was 0.1;
        public static final double I_HOLD = 0.0;
        public static final double D_HOLD = 0.0;
        public static final double F_HOLD = 0.004; // wasg 0.002; //0.004
        public static final double F_HOLD_LOW = 0.004; // wasg 0.002; //was 0.02; //was 0.004

        // Motion Magic constants
        public static final int MAX_VELOCITY = 1500 * 10; // wasg 500 n=ticks/sec * 10 : Elevator velocity (ticks/100ms)
        public static final double SLOW_VELOCITY_PERC = 0.5; //
        public static final int REV_MAX_VELOCITY = 1500 * 10; // wasg 500
        public static final double AUTO_OUTPUT_CONE_HIGH_FAST = 1.0;
        public static final int MAX_ACCELERATION = 20000; // Elevator acceleration (ticks/100ms/s)
        public static final double GRAV_FEED_FORWARD = 400 / 1023.0; // Percent constant to counteract gravity
        public static final double REV_GRAV_FEED_FORWARD = 50 / 1023.0;

        public static final double EL_TICKS_PER_INCH = 1787.61; //was 1750; // PBOT 1700;
        public static final double PID_TOLERANCE = EL_TICKS_PER_INCH/2; // 1/x inch ... allows PID to end without having to be exact
        public static final double ZERO_OFFSET_TICKS = 13.0 * EL_TICKS_PER_INCH; // PBOT 11 // 13 inches above ground

        public static final double LOAD_FROM_DOUBLE_SUBSTATION_POS = 43.16 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS; // n is in inches
        public static final double LOAD_FROM_SINGLE_SUBSTATION_POS = 15 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS;
        public static final double LOAD_FROM_GROUND_POS = 14.215 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS;
        public static final double PLACE_CUBE_HIGH_POS = 44.9 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS;
        public static final double PLACE_CUBE_HIGH_POS_AUTO = 17.0 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS;
        public static final double PLACE_CUBE_MID_POS = 17.0 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS; // was 40
        public static final double PLACE_CONE_HIGH_POS = 46.5 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS;
        public static final double PLACE_CONE_MID_POS = 43 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS; // 36
        public static final double PLACE_CONE_MID_REAR_POS = 23 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS; // 36
        public static final double PLACE_LOW_POS = 19 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS;

        public static final double MAX_HOME_SAFE_POS = 13.75 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS; // was 15.6 // PBOT 14 maximum elevator height to allow arm to come under bar
        public static final double STAGE_POS = MAX_HOME_SAFE_POS;
        public static final double HOME_POS = 14.0 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS; // PBOT 12.5
        public static final double ELEVATOR_30 = 30 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS;

        public static final double MAX_POS = 47 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS;
        public static final double MIN_POS =  13.5 * EL_TICKS_PER_INCH - ZERO_OFFSET_TICKS; // PBOT 12
    }

    public static class TelescopingArm {
        public static final double P = 0.00;
        public static final double FF = 0.00;
        public static final double I = 0.00;
        public static final double D = 0.00;
        public static final double HIGH_VELOCITY = 0.40; // 0.5 // TODO: need more testing
        public static final double MID_VELOCITY = 0.25;
        public static final double LOW_VELOCITY = 0.1; // was 0.2 // TODO: need more testing

        public static final double MAX_POS = 39.8; // 40.5;
        public static final double HIGH_CONE_POS =  39.5;//36.6; // PBOT 38.5; // 40.0; // PBOT 64.0;
        public static final double HIGH_COLLECT_POS =  4.0;//36.6; // PBOT 38.5; // 40.0; // PBOT 64.0;
        public static final double SLOW_POS = 8.0; // 6.0; // PBOT 8.0
        public static final double MID_POS = 2.0; // 6.0; // PBOT 8.0
        public static final double MIN_POS = 1.5; // 2.0; // PBOT 0.002
    }

    public static class Collector {
        public static final double COLLECTOR_RUN_COLLECT = 0.75; // 0.5; //collector percent
        public static final double COLLECTOR_RUN_EJECT = 0.45; // 0.5; //collector percent
    }

    public static class Location {
        public static final int LOAD_FROM_SINGLE_SUBSTATION = 0;
        public static final int LOAD_FROM_DOUBLE_SUBSTATION = 1;
        public static final int LOAD_FROM_FLOOR = 2;

        public static final int PLACE_CONE_HIGH = 3;
        public static final int PLACE_CONE_MID = 4;

        public static final int PLACE_CUBE_HIGH = 5;
        public static final int PLACE_CUBE_MID = 6;
        public static final int PLACE_CONE_MID_REAR = 8;
        public static final int PLACE_LOW = 7;
    }

    public static class LED {
        public static final int NUM_LEDS_TOTAL = 10;
    }
}
