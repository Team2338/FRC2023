package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmMid;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class GoLocation extends CommandBase {
    private int location;
    public GoLocation(int location) {
        super();
        this.location = location;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        /*double armTargetPos;
        double elevatorTargetPos;
        if ( Robot.arm.getPosition() < 30) { // TODO: change to ticks
            new GoLocationFromHome().schedule();
        } else {
            switch (location) {
                case Constants.Location.LOAD_FROM_DOUBLE_SUBSTATION:
                    new SetElevatorPosition(
                            Constants.Elevator.LOAD_FROM_GROUND_POS).schedule();
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
        }*/
        new SetArmMid().schedule();
        new SetElevatorPosition(Constants.Elevator.LOAD_FROM_SINGLE_SUBSTATION_POS).schedule();
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
