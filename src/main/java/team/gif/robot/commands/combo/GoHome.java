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
        if (Math.abs(Robot.elevator.getPosition() - Constants.Elevator.LOAD_FROM_GROUND_POS) < 500 ) {
            System.out.println("Elevator pointing to floor");
            new GoHomeFinal().schedule();
        } else if (Robot.elevator.getPosition() < Constants.Elevator.MAX_HOME_SAFE_POS) {
            System.out.println("Elevator less than 5");
            new SetArmPosition(Constants.Arm.HOME_POS).schedule();
            new SetElevatorPosition(Constants.Elevator.HOME_POS).schedule();
        } else if (Robot.elevator.getPosition() < Constants.Elevator.ELEVATOR_30 ) {
            System.out.println("Elevator less than 30");
            new GoHomePreStage().schedule();
        } else {
            System.out.println("Elevator greater than 30");
            new GoHomeFinal().schedule();
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
