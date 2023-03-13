package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static team.gif.robot.Robot.arm;
import static team.gif.robot.Robot.elevator;
import static team.gif.robot.Robot.telescopingArm;

public class UI {

    public UI() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("FRC2023");

        shuffleboardTab.addBoolean("Manual Cntl", arm::getArmManualFlag)
                .withPosition(2, 0)
                .withSize(1, 1);
        ;
        shuffleboardTab.addNumber("Elevator Ticks", elevator::getPosition)
                .withPosition(1, 0);

        shuffleboardTab.addNumber("Arm Degrees", arm::getPositionDegrees)
                .withPosition(3, 0);

        /*
        *  Everything below here is for debugging
        */
//        shuffleboardTab.addNumber("Arm Ticks", arm::getPosition)
//                .withPosition(0, 0);

//        shuffleboardTab.addNumber("Elevator target", elevator::getTargetPosition)
//                .withPosition(1, 2);

//        shuffleboardTab.addNumber("Arm Output", arm::getOutput)
//                .withPosition(2, 1);
//
//        shuffleboardTab.addNumber("Elevator Output", elevator::getOutputPercent)
//                .withPosition(3, 1);
//
//        shuffleboardTab.addNumber("Arm PID Error", arm::PIDError)
//                .withPosition(0, 1);
//
//        shuffleboardTab.addNumber("Arm target", arm::getTargetPosition)
//                .withPosition(0, 2);
//
//        shuffleboardTab.addNumber("Elevator PID Error", elevator::PIDError)
//                .withPosition(1, 1);
//
//        shuffleboardTab.addNumber("Elevator Inches", elevator::getPositionInches)
//                .withPosition(2, 2);
//
//        shuffleboardTab.addNumber("Tele Pos", telescopingArm::getPosition)
//                .withPosition(4, 0);
//
        shuffleboardTab.addNumber("Gyro Pitch", Robot.pigeon::getPitch)
                .withPosition(5, 0)
                .withSize(2,3);
    }
}
