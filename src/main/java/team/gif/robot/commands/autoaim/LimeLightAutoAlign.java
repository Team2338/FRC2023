package team.gif.robot.commands.autoaim;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.drivers.Limelight;

import java.time.OffsetTime;

public class LimeLightAutoAlign extends CommandBase {

    private final double rTolerence = 3.0;
    private double rOffset;
    private final double yTolerence = 1.0;
    private double yOffset;
    public LimeLightAutoAlign() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.limelight.setLEDMode(Limelight.LED_ON); // turn on - just in case they were turned off somehow
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        double velocity;
        double rotation;

        if (Robot.limelight.hasTarget()) {

            rOffset = Robot.pigeon.getCompassHeading();
            yOffset = -Robot.limelight.getXOffset();

            velocity = (yOffset > 0) ? -0.5 : 0.5;
            rotation = (rOffset > 180) ? 0.2 : -0.2;

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, velocity, rotation);
            SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
            Robot.swervetrain.setModuleStates(moduleStates);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        rOffset = (rOffset > 180) ? (360 - rOffset) : rOffset;
        if (Math.abs(yOffset) < yTolerence && rOffset < rTolerence)
            return true;
        else
            return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0.0);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }
}
