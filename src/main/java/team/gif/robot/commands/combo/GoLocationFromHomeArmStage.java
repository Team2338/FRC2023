package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class GoLocationFromHomeArmStage extends CommandBase {

    public GoLocationFromHomeArmStage() {
        super();
        addRequirements(Robot.arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.arm.move(0.4);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if ( Robot.arm.getPosition() > Constants.Arm.STAGE_POS)
            return true;
        else
            return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.arm.move(0);
        Robot.arm.setTargetPosition(Constants.Arm.STAGE_POS);
    }
}
