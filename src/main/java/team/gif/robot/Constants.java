// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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

        public static final double FRONT_LEFT_OFFSET = 131.748;
        public static final double REAR_LEFT_OFFSET = 20.302;
        public static final double FRONT_RIGHT_OFFSET = 88.682;
        public static final double REAR_RIGHT_OFFSET = 76.904;

        public static final double TRACK_WIDTH = 0.4699;
        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE = 0.4699;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // x was +, y was +
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // x was +, y was -
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), // x was -, y was +
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)); // x was -, y was -

        public static final boolean kGyroReversed = false;

        public static final double MAX_DRIVE_RPM = 3200;

        public static final double MAX_SPEED_METERS_PER_SECOND = MAX_DRIVE_RPM *
            (Math.PI * Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
            (60.0 * Constants.ModuleConstants.GEAR_RATIO);
        public static double kMaxAccelerationMetersPerSecondSquared = 2;// TODO
    }

    public static final class ModuleConstants {
        public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 6 * (2 * Math.PI); //6
        public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 6 * (2 * Math.PI); //7

        public static final double DRIVE_MOTOR_GEAR_RATIO = 2; // TODO: Need to ask Aaron
        public static final double WHEEL_DIAMETER_METERS = 0.10338;
        public static final double DRIVE_ENCODER_ROT_2_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;

        public static final double ENCODER_CPR = 4096.0; //1024
        public static final double kFalconEncoderCPR = 2048;
        public static final double kDriveEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;

        public static final double kTurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) ENCODER_CPR;

        public static final double kPModuleTurningController = 1.0; // 1

        public static final double kPModuleDriveController = 0.3; // 1

        public static final double GEAR_RATIO = 46080.0 / 6720.0; // need to ask aaron

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 5;

        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;
        public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final class DrivetrainPID {
            public static final double frontLeftP = 0.5;
            public static final double frontLeftFF = 0.035;
            public static final double frontRightP = 0.48;
            public static final double frontRightFF = 0.05; //issa good
            public static final double rearLeftP = 0.5;
            public static final double rearLeftFF = 0.035;
            public static final double rearRightP = 0.5;
            public static final double rearRightFF = 0.035;
        }
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;


        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 0.65;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                    MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class Joystick {
        public static final double DEADBAND = 0.1;
    }

    public static class Arm {
        public static final double FF = 0;
        public static final double P = 1.2;
        public static final double I = 0.0005;
        public static final double Ticks_Move = 3;
        public static final double TICKS_ABS_MIN = 1600;
        public static final double TICKS_ABS_MAX = 2800;

    }
    public static class Elevator {
        public static final double P = 4.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.425;
        public static final double REV_F = 0.38;
        public static final double GRAV_FEED_FORWARD = 300 / 1023.0; // Percent constant to counteract gravity
        public static final double REV_GRAV_FEED_FORWARD = 50 / 1023.0;


        public static final int ALLOWABLE_ERROR = 100; // Error to allow move command to end
        public static final int MAX_VELOCITY = 3000; // Elevator velocity (ticks/100ms)
        public static final int REV_MAX_VELOCITY = 4000;
        public static final int MAX_ACCELERATION = 10000; // Elevator acceleration (ticks/100ms/s)

        public static final int PLACE_HIGH_POS = 50000;
        public static final int PLACE_MID_POS = 40000;
        public static final int PLACE_LOW_POS = 8000;
        public static final int LOAD_FROM_GROUND_POS = 5000;
        public static final int LOAD_FROM_DOUBLE_SUBSTATION_POS = 55000;
        public static final int LOAD_FROM_SINGLE_SUBSTATION_POS = 30000;

        public static final int MAX_POS = 60000;
        public static final int MIN_POS =  5000;
    }

    public static class TelescopingArm {
        public static final double TEST = 0.2;
        public static final double MAX_VELOCITY = 0.4; // TODO: need more testing
        public static final double MIN_VELOCITY =
    }

    public static class Collector {
        public static final int CollectorRun = 0; //collector percent
    }
}
