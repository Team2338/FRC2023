package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Globals;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.commands.telescopingArm.ArmMid;
import team.gif.robot.commands.telescopingArm.ArmOut;

/*
 *
 * This CommandBase moves both the arm and elevator to the desired position (exclusive
 *      of home and floor)
 *
 * There are separate classes for moving the mechanisms back home or to the floor.
 * This is because when going home or to the floor, we need to move the
 * mechanisms a different way to avoid crashing the mechanisms into other parts
 * of the robot.
 *
 */
public class GoLocation extends CommandBase {

    private int location;

    public GoLocation(int location) {
        super();
        this.location = location;
    }

    public GoLocation() {
        super();
        this.location = Globals.goLocationTarget; // pull the location set on the previous call to GoLocation
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double armTargetPos;
        double elevatorTargetPos;

        if (Robot.arm.getPositionDegrees() < Constants.Arm.STAGE_POS) { // need to be in a safe place before going anywhere else
            Globals.goLocationTarget = location;
            new GoLocationFromHome().schedule();
        } else {
            switch (location) {
                case Constants.Location.LOAD_FROM_DOUBLE_SUBSTATION:
                    elevatorTargetPos = Constants.Elevator.LOAD_FROM_DOUBLE_SUBSTATION_POS;
                    armTargetPos = Constants.Arm.LOAD_FROM_DOUBLE_SUBSTATION_POS;
                    new ArmIn().schedule();
                    break;
                case Constants.Location.LOAD_FROM_SINGLE_SUBSTATION:
                    elevatorTargetPos = Constants.Elevator.LOAD_FROM_SINGLE_SUBSTATION_POS;
                    armTargetPos = Constants.Arm.LOAD_FROM_SINGLE_SUBSTATION_POS;
                    new ArmIn().schedule();
                    break;
                case Constants.Location.LOAD_FROM_FLOOR:
                    elevatorTargetPos = Constants.Elevator.LOAD_FROM_GROUND_POS;
                    armTargetPos = Constants.Arm.LOAD_FROM_GROUND_POS;
                    new ArmIn().schedule();
                    break;
                case Constants.Location.PLACE_CONE_HIGH:
                    elevatorTargetPos = Constants.Elevator.PLACE_CONE_HIGH_POS;
                    armTargetPos = Constants.Arm.PLACE_CONE_HIGH_POS;
                    new ArmOut(Constants.TelescopingArm.HIGH_POS).schedule();
                    break;
                case Constants.Location.PLACE_CONE_MID:
                    elevatorTargetPos = Constants.Elevator.PLACE_CONE_MID_POS;
                    armTargetPos = Constants.Arm.PLACE_CONE_MID_POS;
                    new ArmMid().schedule();
                    break;
                case Constants.Location.PLACE_CUBE_HIGH:
                    elevatorTargetPos = Constants.Elevator.PLACE_CUBE_HIGH_POS;
                    armTargetPos = Constants.Arm.PLACE_CUBE_HIGH_POS;
                    new ArmIn().schedule();
                    break;
                case Constants.Location.PLACE_CUBE_MID:
                    elevatorTargetPos = Constants.Elevator.PLACE_CUBE_MID_POS;
                    armTargetPos = Constants.Arm.PLACE_CUBE_MID_POS;
                    new ArmIn().schedule();
                    break;
               case Constants.Location.PLACE_LOW:
                    elevatorTargetPos = Constants.Elevator.PLACE_LOW_POS;
                    armTargetPos = Constants.Arm.PLACE_LOW_POS;
                    new ArmIn().schedule();
                    break;
                default:
                    elevatorTargetPos = -1;
                    armTargetPos = -1;
                    break;
            }
            if( armTargetPos >= 0 ) {
                new SetElevatorPosition(elevatorTargetPos).schedule();
                new SetArmPosition(armTargetPos).schedule();
            }
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
