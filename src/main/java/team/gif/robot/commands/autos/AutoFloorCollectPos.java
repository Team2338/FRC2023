// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.Arm;

public class AutoFloorCollectPos extends CommandBase {
    boolean elevatorDone = false;
    boolean elevatorDownDone = false; // need to go to 0 in order to pass arm thru
    boolean armDone = false;
    boolean armStageDone = false;
//    boolean armTeleDone = false;

    double EL_TARGET_POS = Constants.Elevator.LOAD_FROM_GROUND_POS; //+ Robot.elevator.inchesToTicks(4);
    double ARM_TARGET_POS = Constants.Arm.LOAD_FROM_GROUND_POS;// - Robot.arm.degreesToTicks(15);

    double ARM_STAGE_POS = Constants.Arm.STAGE_POS;
//    double ARM_TELE_TARGET_POS = Constants.TelescopingArm.MIN_POS;

    public AutoFloorCollectPos(){
        super();
        addRequirements(Robot.arm,Robot.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevatorDone = false;
        elevatorDownDone = false;
        armDone = false;
        armStageDone= false;
//        armTeleDone = false;

//        Arm.armMotor.configPeakOutputForward(1.0);

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

        // elevator done going down?
        if (elevatorPos < Constants.Elevator.MIN_POS && !elevatorDownDone) {
            System.out.println("AFC: EL Down Done: " + Timer.getFPGATimestamp());
            elevatorDownDone = true;
        }

        // elevator done going up?
        if (elevatorDownDone && elevatorPos > EL_TARGET_POS && !elevatorDone) {
            System.out.println("AFC: EL Up Done: " + Timer.getFPGATimestamp());
            elevatorDone = true;
        }

        // arm in final position?
        if (armPos > ARM_TARGET_POS && !armDone) {
            System.out.println("AFC: Arm Done: " + Timer.getFPGATimestamp());
            Robot.arm.resetI();
            Robot.arm.setTargetPosition(ARM_TARGET_POS);
            Robot.arm.configPeakOutputForward(Constants.Arm.PEAK_OUTPUT_FORWARD);
            Robot.arm.configI(Constants.Arm.I_GT_45);
            armDone = true;
        }

//        if (Robot.telescopingArm.getPosition() < ARM_TELE_TARGET_POS && !armTeleDone) {
//            armTeleDone = true;
//            System.out.println("Tele Done");
//        }

        // move elevator to zero position to allow arm to pass thru
        if (!elevatorDownDone) {
            Robot.elevator.setMotionMagic(Constants.Elevator.MIN_POS, Constants.Elevator.GRAV_FEED_FORWARD);
        }

        // once arm has cleared, lift elevator
        if (elevatorDownDone && armPos > ARM_STAGE_POS && !elevatorDone ) {
            Robot.elevator.setMotionMagic(EL_TARGET_POS, Constants.Elevator.GRAV_FEED_FORWARD);
        }

        if (!armDone) {
            Robot.arm.PIDMove();
        }

//        if (!armTeleDone && armPos > ARM_POS_TEL) {
//            if (Robot.telescopingArm.getPosition() < Constants.TelescopingArm.HIGH_CONE_POS * .80) {
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
            Robot.arm.PIDMove();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return armDone && elevatorDone;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        Arm.armMotor.configPeakOutputForward(Constants.Arm.PEAK_OUTPUT_FORWARD);
    }
}
