// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.Arm;

public class AutoConeMidRear extends CommandBase {
    boolean elevatorDone = false;
    boolean armDone = false;
    boolean armTeleDone = false;
    double elapsedTime = 0;

    double EL_PCH_POS = 23 * Constants.Elevator.EL_TICKS_PER_INCH - Constants.Elevator.ZERO_OFFSET_TICKS;
    double ARM_SLOW_POS = -40.0 * Constants.Arm.TICKS_PER_DEGREE + Constants.Arm.ZERO_OFFSET_TICKS; // was 65
    double ARM_PCH_POS = -50.0 * Constants.Arm.TICKS_PER_DEGREE + Constants.Arm.ZERO_OFFSET_TICKS; // was 65
    double ARM_POS_TELE_START = -15.0 * Constants.Arm.TICKS_PER_DEGREE + Constants.Arm.ZERO_OFFSET_TICKS;
    double ARM_TELE_POS = 35.0;

    public AutoConeMidRear(){
        super();
        addRequirements(Robot.arm,Robot.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.collectorWheels.wheelsOut();
        elevatorDone = false;
        armDone = false;
        armTeleDone = false;
        elapsedTime = Timer.getFPGATimestamp();
        Arm.armMotor.configReverseSoftLimitThreshold(-1700);
        Arm.armMotor.configPeakOutputReverse(-1.0);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double armPos = Robot.arm.getPosition();
        if (armPos < ARM_PCH_POS && !armDone) {
            armDone = true;
        }

        if (Robot.elevator.getPosition() > EL_PCH_POS && !elevatorDone) {
            elevatorDone = true;
        }

        if (Robot.telescopingArm.getPosition() > ARM_TELE_POS && !armTeleDone) {
            armTeleDone = true;
        }

        if (!armDone) {
//            Robot.arm.move(-Constants.Arm.AUTO_OUTPUT_CONE_HIGH_STAGE);
            if( armPos < ARM_SLOW_POS)
                Robot.arm.move(-1.0);
            else
                Robot.arm.move(-0.5);
        }

        if (!elevatorDone && armPos < 500) {
            Robot.elevator.move(Constants.Elevator.AUTO_OUTPUT_CONE_HIGH_FAST);
        }

        if (!armTeleDone && armPos < ARM_POS_TELE_START) {
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
            Robot.arm.setTargetPosition(ARM_PCH_POS);
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
        elapsedTime = Timer.getFPGATimestamp() - elapsedTime;
        System.out.println("New Time: " + elapsedTime);
        elevatorDone = false;
        armDone = false;
        armTeleDone = false;
//        armStageDone = false;
        Arm.armMotor.configReverseSoftLimitThreshold(Constants.Arm.MIN_POS);
    }
}
