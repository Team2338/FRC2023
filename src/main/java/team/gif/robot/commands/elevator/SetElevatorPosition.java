package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.Elevator;

public class SetElevatorPosition extends CommandBase {

    private final double position;

    public SetElevatorPosition(int targetPosition) {
        super();
        // do not allow code to set a point higher or lower than max/min
        if (targetPosition > Constants.Elevator.MAX_POS) { targetPosition = Constants.Elevator.MAX_POS; }
        if (targetPosition < Constants.Elevator.MIN_POS) { targetPosition = Constants.Elevator.MIN_POS; }

        position = targetPosition;

        addRequirements(Robot.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("SetElPos init");
        if (position > Robot.elevator.getPosition()) {
            Robot.elevator.setCruiseVelocity(Constants.Elevator.MAX_VELOCITY);
            Robot.elevator.configF(Constants.Elevator.F);
            Robot.elevator.setMotionMagic(position, Constants.Elevator.GRAV_FEED_FORWARD);
        } else {
            Robot.elevator.setCruiseVelocity(Constants.Elevator.REV_MAX_VELOCITY);
            Robot.elevator.configF(Constants.Elevator.REV_F);
            Robot.elevator.setMotionMagic(position, Constants.Elevator.REV_GRAV_FEED_FORWARD);
        }
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
//        if (!elevator.getFwdLimit() && elevator.getClosedLoopError() < 0) {
//            elevator.setCruiseVelocity(400);
//        } else {
//            elevator.setCruiseVelocity(Constants.Elevator.MAX_VELOCITY);
//        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
//        return Robot.elevator.isFinished();
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("FINISHED TARGET");
    }
}
