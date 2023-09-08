package team.gif.robot.commands.diagnostics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Globals;
import team.gif.robot.Robot;
import team.gif.robot.commands.collector.CollectorCollect;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.collector.WheelsOut;
import team.gif.robot.commands.combo.GoLocation;

public class Check extends CommandBase {
    boolean diagnosticsRunning;
    String check;

    public Check(String check, boolean diagnosticsRunning) {
        this.check = check;
        this.diagnosticsRunning = diagnosticsRunning;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (diagnosticsRunning) {
            switch (check) {
                case "Elevator And Arm":
                    new GoLocation(Constants.Location.PLACE_CONE_HIGH).schedule(); //go to place cone high
                    //check if it reached or not.
                    if (Robot.elevator.getPositionInches() == Constants.Elevator.PLACE_CONE_HIGH_POS &&
                            Robot.arm.getPositionDegrees() == Constants.Arm.PLACE_CONE_HIGH_POS) {
                        Diagnostics.elevatorAndArm = true;
                        Diagnostics.elevatorAndArmProblem = "No Problem";
                    } else {
                        Diagnostics.elevatorAndArm = false;
                        Diagnostics.elevatorAndArmProblem =
                                "elevator pos is not reaching the target, the difference is " + (Constants.Elevator.PLACE_CONE_HIGH_POS - Robot.elevator.getPositionInches())
                                + ". arm pos is not reaching the target, the difference is " + (Constants.Arm.PLACE_CONE_HIGH_POS - Robot.arm.getPositionDegrees());
                    }
                    new WaitCommand(5).andThen(new WheelsIn()).schedule(); //release the cone
                    break;
                case "Collector":
                    new GoLocation(Constants.Location.LOAD_FROM_SINGLE_SUBSTATION).schedule(); //going to single substation
                    new WheelsOut().schedule(); //collector wheels out (cone pos)
                    new CollectorCollect().until(Robot.arm.armGamePieceSensor::get).schedule(); //wait 1s and then run wheels until GP collected, TODO:There is diagnostics code in CollectorCollect().
                    //new GoHome(), //go home pos // TODO: I don't think we need this.
                    break;
            }
        }

    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}