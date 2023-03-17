package team.gif.robot.commands.autoaim;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

import java.time.OffsetDateTime;

public class LimeLightAutoRotate extends CommandBase {
    private final double rTolerence = 1.0; // degrees
    private double rOffset;
    private final double yTolerence = 1.0; // degrees
    private double yOffset;
    private int executeCount;

    public LimeLightAutoRotate() {addRequirements(Robot.swervetrain);}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.limelightLow.setLEDOn();
        executeCount = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double rotationSpeed = 0;

        if (executeCount++ < 3) return; // wait for LED to turn on

        if (Robot.limelightLow.hasTarget()) {
            yOffset = -Robot.limelightLow.getXOffset();
            rotationSpeed = (Math.abs(yOffset) < yTolerence) ? 0 : ((yOffset > 0) ? -0.2 : 0.2);
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(Math.abs(rotationSpeed),0,rotationSpeed); //turns towards xoffset and drives forward
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.limelightLow.setLEDOff();
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0.0);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }
}
