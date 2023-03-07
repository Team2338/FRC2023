package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class MoveCloserSlow extends CommandBase {

    public MoveCloserSlow() {
        addRequirements(Robot.swervetrain); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
//        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, -0.20, 0.0);
//        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
//        Robot.swervetrain.setModuleStates(moduleStates);
        if( Robot.oi.driver.getHID().getRightStickButton())
            Robot.swervetrain.drive(-0.5, 0, 0.0);
        else
            Robot.swervetrain.drive(-0.2, 0, 0.0);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
//        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
//        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
//        Robot.swervetrain.setModuleStates(moduleStates);
        Robot.swervetrain.drive(0.0, 0.0, 0.0);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }
}
