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
        shuffleboardTab.addBoolean("Enable Indexer", () -> Robot.arm.armManualFlag)
                .withPosition(3, 1)
                .withSize(1, 1);
        ;
        shuffleboardTab.addNumber("Arm Ticks", arm::getPosition)
                .withPosition(1, 1);
        shuffleboardTab.addNumber("Elevator Ticks", elevator::getPosition)
                .withPosition(2, 1);

    }
}
