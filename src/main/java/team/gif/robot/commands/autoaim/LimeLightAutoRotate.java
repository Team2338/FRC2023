package team.gif.robot.commands.autoaim;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.drivers.Limelight;

public class LimeLightAutoRotate extends CommandBase {

    private final double RotationTolerence = 1.0;
    private double RotationOffset;
    public LimeLightAutoRotate() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.limelight.setLEDMode(Limelight.LED_ON); // turn on - just in case they were turned off somehow
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        double Rotation;

        if (Robot.limelight.hasTarget()) {

            RotationOffset = Robot.pigeon.getCompassHeading();

            Rotation = (RotationOffset > 180) ? 0.2 : -0.2;

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, Rotation);
            SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
            Robot.swervetrain.setModuleStates(moduleStates);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if (RotationOffset > 180) Math.abs(360 - RotationOffset);

        if (RotationOffset < RotationTolerence)
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
