// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AutoHoldLowRearPos extends CommandBase {

    double EL_TARGET_POS = 20 * Constants.Elevator.EL_TICKS_PER_INCH - Constants.Elevator.ZERO_OFFSET_TICKS;
    double ARM_TARGET_POS = Robot.arm.degreesToPos(-95);

    public AutoHoldLowRearPos() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("AutoHoldRearPos Starting");
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (!Robot.runningAutonomousMode){
            Robot.elevator.setElevatorTargetPos(EL_TARGET_POS);
            Robot.elevator.PIDHold();

            Robot.arm.setTargetPosition(ARM_TARGET_POS);
            Robot.arm.configPeakOutputForward(Constants.Arm.PEAK_OUTPUT_FORWARD);
            Robot.arm.configI(Constants.Arm.I_GT_45);
            Robot.arm.resetI();
            Robot.arm.PIDMove();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if (Robot.runningAutonomousMode) {
            return true;
        }

        boolean dX = Robot.oi.dX.getAsBoolean();
        return (dX); // need a kill switch
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        System.out.println("AutoHoldRearPos finished");
    }
}
