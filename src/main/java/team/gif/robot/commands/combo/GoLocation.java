package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmMid;
import team.gif.robot.commands.arm.SetArmPosition;
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
        double armTargetPos = -1;
        int elevatorTargetPos = -1;

//        if ( Robot.arm.getPosition() < 30) { // TODO: change to ticks
//            new GoLocationFromHome().schedule();
//        } else {
            switch (location) {
                case Constants.Location.LOAD_FROM_DOUBLE_SUBSTATION:
                    elevatorTargetPos = Constants.Elevator.LOAD_FROM_DOUBLE_SUBSTATION_POS;
                    armTargetPos = Constants.Arm.LOAD_FROM_DOUBLE_SUBSTATION_POS;
                    break;

                case Constants.Location.LOAD_FROM_SINGLE_SUBSTATION:
                    elevatorTargetPos = Constants.Elevator.LOAD_FROM_SINGLE_SUBSTATION_POS;
                    armTargetPos = Constants.Arm.LOAD_FROM_SINGLE_SUBSTATION_POS;
                    break;
/*                case Constants.Location.LOAD_FROM_FLOOR:
                    new SetElevatorPosition(
                            Constants.Elevator.LOAD_FROM_DOUBLE_SUBSTATION_POS).schedule();
                    break;
                case Constants.Location.PLACE_CONE_HIGH:
                    new SetElevatorPosition(
                            Constants.Elevator.PLACE_CONE_HIGH_POS).schedule();
                    break;
                case Constants.Location.PLACE_CONE_MID:
                    new SetElevatorPosition(
                            Constants.Elevator.PLACE_CONE_MID_POS).schedule();
                    break;
                case Constants.Location.PLACE_CUBE_HIGH:
                    new SetElevatorPosition(
                            Constants.Elevator.PLACE_CUBE_HIGH_POS).schedule();
                    break;
*/                case Constants.Location.PLACE_CUBE_MID:
                    elevatorTargetPos = Constants.Elevator.PLACE_CUBE_MID_POS;
                    armTargetPos = Constants.Arm.PLACE_CUBE_MID_POS;
                    break;
               case Constants.Location.PLACE_LOW:
                    elevatorTargetPos = Constants.Elevator.PLACE_LOW_POS;
                    armTargetPos = Constants.Arm.PLACE_LOW_POS;
                    break;
//            }
//        }
        }
        if( armTargetPos >= 0 ) {
            new SetElevatorPosition(elevatorTargetPos).schedule();
            new SetArmPosition(armTargetPos).schedule();
        }
//        new SetArmMid().schedule();
//        new SetElevatorPosition(Constants.Elevator.LOAD_FROM_SINGLE_SUBSTATION_POS).schedule();
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
