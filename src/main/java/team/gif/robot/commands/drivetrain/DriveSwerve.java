package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class DriveSwerve extends CommandBase {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turnLimiter;

    public DriveSwerve() {
        this.xLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turnLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        addRequirements(Robot.swervetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (Robot.isCompBot) {
            double x = Robot.oi.driver.getLeftX();
            x = (Math.abs(x) > Constants.Joystick.DEADBAND) ? x : 0.0;
            double y = -Robot.oi.driver.getLeftY();
            y = (Math.abs(y) > Constants.Joystick.DEADBAND) ? y : 0.0; //0.00001;
            double rot = Robot.oi.driver.getRightX();
            rot = (Math.abs(rot) > Constants.Joystick.DEADBAND) ? rot : 0.0;

            //Forward speed, Sideways speed, Rotation Speed
            x = xLimiter.calculate(x) * Constants.ModuleConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
            y = yLimiter.calculate(y) * Constants.ModuleConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
            rot = turnLimiter.calculate(rot) * Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

            // the robot starts facing the driver station so for this year negating y and x
            Robot.swervetrain.drive(y, x, rot);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
