package team.gif.robot.commands.telescopingArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class MotorRev extends CommandBase {
    public MotorRev() {
        super();
        addRequirements(Robot.telescopingArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.telescopingArm.getVelocity() == Constants.TelescopingArm.MAX_VELOCITY) {
            Robot.telescopingArm.setTelescopingMotor(0);
        } else {
            Robot.telescopingArm.setTelescopingMotor(-Constants.TelescopingArm.TEST);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.telescopingArm.setTelescopingMotor(0);
    }
}
