package team.gif.robot.commands.diagnostics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Globals;
import team.gif.robot.Robot;

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
        Globals.diagnosticsFlag = true;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (diagnosticsRunning) {
            switch (check) {
                case "Elevator And Arm":
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