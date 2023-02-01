package team.gif.robot;

public abstract class RobotMap {
    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int AUX_CONTROLLER_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;

    // Drivetrain
    public static final int RIGHT_DRIVETRAIN_ONE = 11;
    public static final int RIGHT_DRIVETRAIN_TWO = 12;
    public static final int LEFT_DRIVETRAIN_ONE = 21;
    public static final int LEFT_DRIVETRAIN_TWO = 22;

    // Arm
    public static final int ARM_MOTOR = 5; //30 TODO
    public static final int ARM_ENCODER = 6; //31 TODO

    // Pigeon
    public static final int PIGEON = 0;

    // Elevator
    public static final int ELEVATOR_MOTOR_ID = 41;
}
