// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.Arm;

public class AutoStagePos extends CommandBase {
    boolean armDone = false;
    boolean elevatorDownDone = false;

    double ARM_TARGET_POS = Constants.Arm.STAGE_POS;
    double EL_TARGET_POS = Constants.Elevator.MIN_POS;

    public AutoStagePos(){
        super();
        addRequirements(Robot.arm, Robot.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armDone = false;
        elevatorDownDone = false;
        Arm.armMotor.configReverseSoftLimitThreshold(-1700);
        Arm.armMotor.configPeakOutputReverse(-0.8); // arm is forward, coming back, need to slow it down

        // set this here in case auto ends before this does
        Robot.elevator.setElevatorTargetPos(EL_TARGET_POS);
        Robot.arm.setTargetPosition(ARM_TARGET_POS);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double armPos = Robot.arm.getPosition();
        double elPos = Robot.elevator.getPosition();

        if (armPos > ARM_TARGET_POS && !armDone) {
            armDone = true;
        }

        if (elPos < EL_TARGET_POS && !elevatorDownDone) {
            elevatorDownDone = true;
        }

        if (!armDone) {
            Robot.arm.move(1.0);
        }

        if (!elevatorDownDone) {
            Robot.elevator.move(-Constants.Elevator.SLOW_VELOCITY_PERC);
        }

        if (armDone) {
            // hold arm at position
            Robot.arm.setTargetPosition(ARM_TARGET_POS);
            Robot.arm.configPeakOutputForward(Constants.Arm.PEAK_OUTPUT_FORWARD);
            Robot.arm.configI(Constants.Arm.I_GT_45);
            Robot.arm.resetI();
            Robot.arm.PIDMove();
        }
        if (elevatorDownDone) {
            Robot.elevator.PIDHold();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return armDone && elevatorDownDone;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        Robot.elevator.setElevatorTargetPos(EL_TARGET_POS);
        Robot.arm.setTargetPosition(ARM_TARGET_POS);
        Arm.armMotor.configReverseSoftLimitThreshold(Constants.Arm.MIN_POS);
        Arm.armMotor.configPeakOutputReverse(Constants.Arm.PEAK_OUTPUT_REVERSE);
        System.out.println("Arm at stage pos: done");
    }
}
