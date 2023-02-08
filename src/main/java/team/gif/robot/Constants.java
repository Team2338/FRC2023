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

        public static final double kFrontLeftOffset = -1581.0; // TODO: Calculate 454.0
        public static final double kRearLeftOffset = 1231.0; //-2853.0
        public static final double kFrontRightOffset = -55.0; //-35.0
        public static final double kRearRightOffset = 87.0;

        public static final double kTrackWidth = 0.4699;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.4699;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // x was +, y was +
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // x was +, y was -
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // x was -, y was +
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); // x was -, y was -

        public static final boolean kGyroReversed = false;

        public static final double kMaxDriveRPM = 4800;

        public static final double kMaxSpeedMetersPerSecond = kMaxDriveRPM *
            (Math.PI * Constants.ModuleConstants.kWheelDiameterMeters) /
            (60.0 * Constants.ModuleConstants.kGearRatio);
        public static double kMaxAccelerationMetersPerSecondSquared = 2;// TODO
    }

    public static final class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 6 * (2 * Math.PI); //6
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 6 * (2 * Math.PI); //7

        public static final double kDriveMotorGearRatio = 2; // TODO: Need to ask Aaron
        public static final double kWheelDiameterMeters = 0.10338;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;

        public static final double kEncoderCPR = 4096.0; //1024
        public static final double kFalconEncoderCPR = 2048;
        public static final double kDriveEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final double kTurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) kEncoderCPR;

        public static final double kPModuleTurningController = 1.0; // 1

        public static final double kPModuleDriveController = 0.3; // 1

        public static final double kGearRatio = 46080.0 / 6720.0; // need to ask aaron

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;

        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;

        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;


        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 0.65;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class Joystick {
        public static final double DEADBAND = 0.1;
    }

    public static class Arm {
        public static final double FF = 0;
        public static final double P = 1.2;
        public static final double I = 0.0005;
        public static final double Ticks_Move = 3;
        public static final double TICKS_ABS_MIN = 1300;
        public static final double TICKS_ABS_MAX = 3000;
        public static final int PID_TOLERANCE = 30; // allows PID to get closer

    }
    public static class Elevator {
        public static final double P = 4.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.425;
        public static final double REV_F = 0.38;
        public static final double GRAV_FEED_FORWARD = 300 / 1023.0; // Percent constant to counteract gravity
        public static final double REV_GRAV_FEED_FORWARD = 50 / 1023.0;


        public static final int PID_TOLERANCE = 50; // allows PID to get closer
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

    public static class Collector {
        public static final int CollectorRun = 0; //collector percent
    }

    public static class Location {
        public static final int LOAD_FROM_SINGLE_SUBSTATION = 0;
        public static final int LOAD_FROM_DOUBLE_SUBSTATION = 1;

        public static final int PLACE_CONE_MID = 3;
        public static final int PLACE_CONE_HIGH = 4;

        public static final int PLACE_CUBE_HIGH = 5;
        public static final int PLACE_CUBE_MID = 6;
        public static final int PLACE_CUBE_LOW = 7;
    }
}
