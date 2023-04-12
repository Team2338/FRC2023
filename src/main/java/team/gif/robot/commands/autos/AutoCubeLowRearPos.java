// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.Arm;

public class AutoCubeLowRearPos extends CommandBase {
    boolean elevatorDone = false;
    boolean elevatorDownDone = false;
    boolean armDone = false;
    boolean armStageDone = false;
//    boolean armTeleDone = false;

    double EL_TARGET_POS = 20 * Constants.Elevator.EL_TICKS_PER_INCH - Constants.Elevator.ZERO_OFFSET_TICKS;
    double ARM_TARGET_POS = Robot.arm.degreesToPos(-95);
//    double ARM_POS_TELE_START = Robot.arm.degreesToPos(-85);

//    double TELE_TARGET_POS = 38.0;
    double ARM_STAGE_POS = Constants.Arm.STAGE_POS;


    public AutoCubeLowRearPos(){
        super();
        addRequirements(Robot.arm,Robot.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("AutoCubeLowRear");
        elevatorDone = false;
        elevatorDownDone = false;
        armDone = false;
        armStageDone = false;
//        armTeleDone = false;
        Arm.armMotor.configReverseSoftLimitThreshold(-1700);
        Arm.armMotor.configPeakOutputReverse(-1.0);

        Robot.arm.setTargetPosition(ARM_TARGET_POS);
        Robot.arm.configPeakOutputForward(Constants.Arm.PEAK_OUTPUT_FORWARD);
        Robot.arm.configI(Constants.Arm.I_GT_45);
        Robot.arm.resetI();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double armPos = Robot.arm.getPosition();
        double elevatorPos = Robot.elevator.getPosition();
//        double armTelePos = Robot.telescopingArm.getPosition();

        if (elevatorPos < Constants.Elevator.MIN_POS && !elevatorDownDone) {
            System.out.println("ACLRP: EL Down Done: " + Timer.getFPGATimestamp());
            elevatorDownDone = true;
        }

        if (elevatorDownDone && elevatorPos > EL_TARGET_POS && !elevatorDone) {
            elevatorDone = true;
            System.out.println("ACLRP: Elevator Done: " + Timer.getFPGATimestamp());
        }

        if (armPos < ARM_TARGET_POS && !armDone) {
            System.out.println("ACLRP: Arm Done " + String.format("%.2f",Robot.arm.getPositionDegrees()) + " degrees, time: " + Timer.getFPGATimestamp());
            armDone = true;
        }

//        if (armTelePos > TELE_TARGET_POS && !armTeleDone) {
//            armTeleDone = true;
//            System.out.println("CMR: Tele Done");
//        }

        if (!armDone) {
            Robot.arm.PIDMove();
        }

        if (!elevatorDownDone) {
            Robot.elevator.move(-Constants.Elevator.SLOW_VELOCITY_PERC);
        }

        if (elevatorDownDone && armPos < Robot.arm.degreesToTicks(0) && !elevatorDone) {
            Robot.elevator.setMotionMagic(EL_TARGET_POS, Constants.Elevator.GRAV_FEED_FORWARD);
        }

//        if (!armTeleDone && armPos < ARM_POS_TELE_START) {
//            if (armTelePos < TELE_TARGET_POS - 6) {
//                Robot.telescopingArm.setMotorSpeed(Constants.TelescopingArm.HIGH_VELOCITY);
//            } else {
//                Robot.telescopingArm.setMotorSpeed(Constants.TelescopingArm.LOW_VELOCITY);
//            }
//        }

        if (elevatorDone) {
            Robot.elevator.setElevatorTargetPos(EL_TARGET_POS);
            Robot.elevator.PIDHold();
        }

//        if (armTeleDone) {
//            Robot.telescopingArm.setMotorSpeed(0);
//        }

        if (armDone) {
            // hold arm at position
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
        return armDone && elevatorDownDone && elevatorDone ;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        System.out.println("ending ACLRP");
        Arm.armMotor.configReverseSoftLimitThreshold(Constants.Arm.MIN_POS);
    }
}
