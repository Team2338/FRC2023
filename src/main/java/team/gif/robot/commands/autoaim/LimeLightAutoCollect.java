package team.gif.robot.commands.autoaim;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.collector.CollectorCollect;

public class LimeLightAutoCollect extends CommandBase {
    private final double rTolerence = 1.0; // degrees
    private double rOffset;
    private final double yTolerence = 5.0; // degrees
    private double yOffset;

    int count = 0;

    public LimeLightAutoCollect() {addRequirements(Robot.swervetrain);}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        new CollectorCollect().schedule();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double rotationSpeed = 0;
        if (count < 20) {
            if (Robot.limelightHigh.hasTarget()) {
                yOffset = -Robot.limelightHigh.getXOffset();
                rotationSpeed = (Math.abs(yOffset) < yTolerence) ? 0 : ((yOffset > 0) ? -0.4 : 0.4);
                if (rotationSpeed == 0) {
                    count++;
                } else {
                    count = 0;
                }
            }

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,rotationSpeed); //turns towards xoffset and drives forward
            SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
            Robot.swervetrain.setModuleStates(moduleStates);
        }
        if (count == 20) {
            Robot.telescopingArm.setMotorSpeed(Constants.TelescopingArm.LOW_VELOCITY);
        }

    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.arm.armGamePieceSensor.get();
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0.0);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }
}
