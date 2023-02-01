// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class Arm {
        public static final double FF = 0;
        public static final double P = 1.2;
        public static final double I = 0.0005;
        public static final double Ticks_Move = 3;
        public static final double TICKS_ABS_MAX = 3000;
        public static final double TICKS_ABS_MIN = 500;

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
        public static final int MAX_VELOCITY = 1700; // Elevator velocity (ticks/100ms)
        public static final int REV_MAX_VELOCITY = 2800;
        public static final int MAX_ACCELERATION = 8000; // Elevator acceleration (ticks/100ms/s)

        public static final int COLLECT_FROM_GROUND_POS = 200;
        public static final int LOAD_FROM_SINGLE_SUBSTATION_POS = 300;
        public static final int LOAD_FROM_DOUBLE_SUBSTATION_POS = 400;
        public static final int PLACE_HIGH_POS = 500;
        public static final int PLACE_MID_POS = 300;
        public static final int PLACE_LOW_POS = 200;

        public static final int MIN_POS = 100;
        public static final int MAX_POS = 1000;
    }
}
