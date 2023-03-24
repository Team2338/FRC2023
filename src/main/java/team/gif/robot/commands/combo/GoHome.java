package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class GoHome extends CommandBase {
    public GoHome() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Math.abs(Robot.elevator.getPosition() - Constants.Elevator.LOAD_FROM_GROUND_POS) < Constants.Elevator.EL_TICKS_PER_INCH &&
                (Robot.arm.getPosition() > Constants.Arm.ARM_80)) {
            //System.out.println("Elevator/Arm pointing to floor, move arm and then stage and home");
            new GoHomeArmStageHome().schedule();
        } else if (Robot.elevator.getPosition() < Constants.Elevator.MAX_HOME_SAFE_POS) {
            //System.out.println("Elevator within safe position to move arm");
            new SetArmPosition(Constants.Arm.STAGE_POS).schedule();
            // only move if we are not stage already
            if( Math.abs(Robot.elevator.getPosition() - Constants.Elevator.STAGE_POS) > Constants.Elevator.PID_TOLERANCE)
                new SetElevatorPosition(Constants.Elevator.STAGE_POS).schedule();
        } else if (Robot.elevator.getPosition() < Constants.Elevator.ELEVATOR_30 ) {
            //System.out.println("Elevator less than 30, go to stage position then home");
            new GoHomeStageHome().schedule();
        } else {
            //System.out.println("Elevator greater than 30, move arm to safe 80 degrees, then stage and home");
            new GoHomeArmStageHome().schedule();
        }
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
