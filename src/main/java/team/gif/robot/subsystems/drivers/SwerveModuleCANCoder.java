package team.gif.robot.subsystems.drivers;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team.gif.robot.Constants;

public class SwerveModuleCANCoder {

    private final WPI_TalonSRX turnMotor;
    private final CANCoder canCoder;
    private final CANSparkMax driveMotor;

    private final double kFF;
    private final double kP;

    private final boolean isAbsInverted;

    private double turningOffset;

    private final ProfiledPIDController turningPID;

//    private final PIDController drivePID =
//            new PIDController(Constants.ModuleConstants.kPModuleDriveController, 0, 0);


    /**
     * Constructor for a TalonSRX, NEO based Swerve Module
     * @param driveMotor NEO motor channel ID
     * @param turnMotor TalonSRX motor channel ID
     * @param isTurningInverted Boolean for if the motor turning the axle is inverted
     * @param isDriveInverted Boolean for if the motor driving the wheel is inverted
     * @param isAbsInverted Boolean for if the absolute encoder checking turn position is inverted
     * @param turningOffset Difference between the absolute encoder and the encoder on the turnMotor
     */
    public SwerveModuleCANCoder (
        int driveMotor,
        int turnMotor,
        boolean isTurningInverted,
        boolean isDriveInverted,
        boolean isAbsInverted,
        double turningOffset,
        int canCoder,
        double kFF,
        double kP
    ) {
        this.driveMotor = new CANSparkMax(driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.turnMotor = new WPI_TalonSRX(turnMotor);

        this.driveMotor.restoreFactoryDefaults();
        this.turnMotor.configFactoryDefault();

        this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake); //TODO: Need confirmation on mode
        this.turnMotor.setNeutralMode(NeutralMode.Brake);

        this.driveMotor.setInverted(isDriveInverted);
        this.turnMotor.setInverted(isTurningInverted);
        this.isAbsInverted = isAbsInverted;

        this.turningOffset = turningOffset;

        this.driveMotor.getEncoder().setPositionConversionFactor(Constants.ModuleConstants.DRIVE_ENCODER_ROT_2_METER);
        this.driveMotor.getEncoder().setVelocityConversionFactor(Constants.ModuleConstants.DRIVE_ENCODER_RPM_2_METER_PER_SEC);

        this.canCoder = new CANCoder(canCoder);
        this.canCoder.configFactoryDefault();
        this.canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        this.kFF = kFF;
        this.kP = kP;

        this.turningPID =  new
            ProfiledPIDController(kP, 0, 0,
            new TrapezoidProfile.Constraints(
                Constants.ModuleConstants.MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND, Constants.ModuleConstants.MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
            ));
        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        this.driveMotor.setSmartCurrentLimit(20, 40);
    }

    public CANSparkMax getDriveMotor() {
        return this.driveMotor;
    }

    public TalonSRX getTurnMotor() {
        return this.turnMotor;
    }

    /**
     * Get the active state of the swerve module
     * @return Returns a SwerveModuleState of the drive velocity and turn velocity
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnVelocity()));
    }

    /**
     * Get the active drive velocity
     * @return Returns the active drive velocity as a double in RPM
     */
    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    /**
     * Get the active turn velocity
     * @return Returns the active turn velocity as a double in EncoderTicks per 100ms
     */
    public double getTurnVelocity() {
        return canCoder.getVelocity();
    }

    /**
     * Get the heading of the canCoder - will also include the offset
     * @return Returns the raw heading of the canCoder (deg)
     */
    public double getRawHeading() {
        return canCoder.getAbsolutePosition();
    }

    /**
     * Get the heading of the swerve module
     * @return Returns the heading of the module in radians as a double
     */
    public double getTurningHeading() {
        double heading = Units.degreesToRadians(getRawHeading() - this.turningOffset);
        heading *= isAbsInverted ? -1.0 : 1.0;
        heading %= 2 * Math.PI;
        return heading;
    }

    /**
     * Reset the wheels to their 0 positions
     */
    public void resetWheel() {
        final double error = getTurningHeading();
        final double kff = kFF * Math.abs(error) / error;
        final double turnOutput = kff + (kP * error);

        turnMotor.set(turnOutput);
    }

    /**
     * Find the reverse of a given angle (i.e. pi/4->7pi/4)
     * @param radians the angle in radians to reverse
     * @return the reversed angle
     */
    private double findRevAngle(double radians) {
        return (Math.PI * 2 + radians) % (2 * Math.PI) - Math.PI;
    }

    /**
     * Finds the distance in ticks between two setpoints
     * @param setpoint initial/current point
     * @param position desired position
     * @return the distance between the two point
     */
    private double getDistance(double setpoint, double position) {
        return Math.abs(setpoint - position);
    }

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

    /**
     * Set the desired state of the swerve module
     * @param state The desired state of the swerve module
     */
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState stateOptimized = optimizeState(state);
        double driveOutput = stateOptimized.speedMetersPerSecond / Constants.Drivetrain.MAX_SPEED_METERS_PER_SECOND;
        final double error = getTurningHeading() - stateOptimized.angle.getRadians();
        final double kff = kFF * Math.abs(error) / error;
        final double turnOutput = kff + (kP * error);
        driveMotor.set(driveOutput);
        turnMotor.set(turnOutput);
    }

    /**
     * Stop the swerve modules
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /**
     * Zeros all the SwerveModule encoders.
     */
    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
//        canCoder.setPosition(0);
    }

    /**
     * Get the position of the swerve module - TODO: HAS BUG
     * @return the position of the swerve module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), new Rotation2d(getTurningHeading()));
    }

    /**
     * Resets the drive encoder
     */
    public void resetDriveEncoders() {
        driveMotor.getEncoder().setPosition(0.0);
    }
}
