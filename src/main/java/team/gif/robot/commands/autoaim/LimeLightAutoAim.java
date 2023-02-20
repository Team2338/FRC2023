package team.gif.robot.commands.autoaim;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.drivers.Limelight;


public class LimeLightAutoAim extends CommandBase {

    private final double yTolerence = 1.0;
    private double yOffset;
    public LimeLightAutoAim() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.limelight.setLEDMode(Limelight.LED_ON); // turn on - just in case they were turned off somehow
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        double velocity;

        if (Robot.limelight.hasTarget()) {

            yOffset = -Robot.limelight.getXOffset();
            velocity = (yOffset > 0) ? -0.5 : 0.5;
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, velocity, 0.0);
            SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
            Robot.swervetrain.setModuleStates(moduleStates);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if (Math.abs(yOffset) < yTolerence)
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
