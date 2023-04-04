// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AutoConeHighFast extends CommandBase {
    boolean elevatorDone = false;
    boolean armStageDone = false;
    boolean armDone = false;
    boolean armTeleDone = false;
    double elapsedTime = 0;

    double EL_PCH_POS = Constants.Elevator.PLACE_CONE_HIGH_POS - (5 * Constants.Elevator.EL_TICKS_PER_INCH);
    double ARM_PCH_POS = Constants.Arm.PLACE_CONE_HIGH_POS - (15.0 * Constants.Arm.TICKS_PER_DEGREE);

    public AutoConeHighFast(){
        super();
        addRequirements(Robot.arm,Robot.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevatorDone = false;
        armDone = false;
        armTeleDone = false;
        armStageDone = false;
        elapsedTime = Timer.getFPGATimestamp();
        Robot.collectorWheels.wheelsOut();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double armPos = Robot.arm.getPosition();

        if (armPos > (Constants.Arm.STAGE_POS + (5.0 * Constants.Arm.TICKS_PER_DEGREE))) {
            armStageDone = true;
        }

        if (armPos > ARM_PCH_POS) {
            armDone = true;
        }

        if (Robot.elevator.getPosition() > EL_PCH_POS) {
            elevatorDone = true;
        }

        if (Robot.telescopingArm.getPosition() > Constants.TelescopingArm.HIGH_CONE_POS ) {
            armTeleDone = true;
        }

        if (!armDone) {
            if( !armStageDone)
                Robot.arm.move(Constants.Arm.AUTO_OUTPUT_CONE_HIGH_STAGE);
            else
                Robot.arm.move(Constants.Arm.AUTO_OUTPUT_CONE_HIGH_FAST);
        }

        if (armStageDone && !elevatorDone) {
            Robot.elevator.move(Constants.Elevator.AUTO_OUTPUT_CONE_HIGH_FAST);
        }

        if (armStageDone && !armTeleDone) {
            if (Robot.telescopingArm.getPosition() < Constants.TelescopingArm.HIGH_CONE_POS * .80) {
                Robot.telescopingArm.setMotorSpeed(Constants.TelescopingArm.HIGH_VELOCITY);
            } else {
                Robot.telescopingArm.setMotorSpeed(Constants.TelescopingArm.LOW_VELOCITY);
            }
        }

        if (elevatorDone) {
            Robot.elevator.setElevatorTargetPos(Robot.elevator.getPosition());
            Robot.elevator.PIDHold();
        }

        if (armTeleDone) {
            Robot.telescopingArm.setMotorSpeed(0);
        }

        if (armDone) {
            // hold arm at position
            Robot.arm.setTargetPosition(Constants.Arm.PLACE_CONE_HIGH_POS - (5.0 * Constants.Arm.TICKS_PER_DEGREE));
            Robot.arm.configPeakOutputForward(Constants.Arm.PEAK_OUTPUT_FORWARD);
            Robot.arm.configI(Constants.Arm.I_GT_45);
            Robot.arm.resetI();
            Robot.arm.PIDMove();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return armDone && elevatorDone && armTeleDone;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        Robot.collectorWheels.wheelsIn();
        elapsedTime = Timer.getFPGATimestamp() - elapsedTime;
        System.out.println("New Time: " + elapsedTime);
        elevatorDone = false;
        armDone = false;
        armTeleDone = false;
        armStageDone = false;
    }
}
