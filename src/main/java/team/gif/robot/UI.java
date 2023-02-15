package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static team.gif.robot.Robot.arm;
import static team.gif.robot.Robot.elevator;

public class UI {

    public UI() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("FRC2023");

//        shuffleboardTab.add("Arm Ticks", arm.getPosition()); // displaying the ticks for the arm.
//        shuffleboardTab.add("Elevator Ticks", elevator.getPosition()); // displaying the ticks for the arm.
//        shuffleboardTab.addBoolean("Enable Indexer", () -> Robot.arm.armManualFlag)
        shuffleboardTab.addBoolean("Manual Cntl", arm::getArmManualFlag)
                .withPosition(2, 0)
                .withSize(1, 1);
        ;
        shuffleboardTab.addNumber("Arm Ticks", arm::getPosition)
                .withPosition(0, 0);
        shuffleboardTab.addNumber("Elevator Ticks", elevator::getPosition)
                .withPosition(1, 0);

        shuffleboardTab.addNumber("Elevator target", elevator::getTargetPosition)
                .withPosition(1, 2);

        shuffleboardTab.addNumber("Arm Output", arm::getOutput)
                .withPosition(2, 1);

        shuffleboardTab.addNumber("Arm PID Error", arm::PIDError)
                .withPosition(0, 1);

        shuffleboardTab.addNumber("Arm target", arm::getTargetPosition)
                .withPosition(0, 2);

        shuffleboardTab.addNumber("Elevator PID Error", elevator::PIDError)
                .withPosition(1, 1);

        shuffleboardTab.addNumber("Arm Degrees", arm::getPositionDegrees)
                .withPosition(3, 0);
    }
}
