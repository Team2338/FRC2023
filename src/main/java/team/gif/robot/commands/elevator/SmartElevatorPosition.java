package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;


public class SmartElevatorPosition extends CommandBase {

    public enum Location {
        COLLECT_FROM_GROUND,
        LOAD_FROM_SINGLE_SUBSTATION,
        LOAD_FROM_DOUBLE_SUBSTATION,
        PLACE_HIGH,
        PLACE_MID,
        PLACE_LOW
    };

    private final Location position;

    public SmartElevatorPosition(Location position) {
        super();
        this.position = position;
        addRequirements(Robot.elevator);
    }

    @Override
    public void initialize() {
        switch (position) {
            case COLLECT_FROM_GROUND:
                new SetElevatorPosition(
                        Constants.Elevator.COLLECT_FROM_GROUND_POS).schedule();
                break;
            case LOAD_FROM_SINGLE_SUBSTATION:
                new SetElevatorPosition(
                        Constants.Elevator.LOAD_FROM_SINGLE_SUBSTATION_POS).schedule();
                break;
            case LOAD_FROM_DOUBLE_SUBSTATION:
                new SetElevatorPosition(
                        Constants.Elevator.LOAD_FROM_DOUBLE_SUBSTATION_POS).schedule();
                break;
            case PLACE_HIGH:
                new SetElevatorPosition(
                        Constants.Elevator.PLACE_HIGH_POS).schedule();
                break;
            case PLACE_MID:
                new SetElevatorPosition(
                        Constants.Elevator.PLACE_MID_POS).schedule();
                break;
            case PLACE_LOW:
                new SetElevatorPosition(
                        Constants.Elevator.PLACE_LOW_POS).schedule();
                break;
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
