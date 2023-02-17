package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static team.gif.robot.Robot.arm;
import static team.gif.robot.Robot.drivetrain;
import static team.gif.robot.Robot.elevator;
import static team.gif.robot.Robot.isCompBot;
import static team.gif.robot.Robot.isTankPBot;

public class UI {

    public UI() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("FRC2023");

        shuffleboardTab.add("Arm Ticks", arm.getTicks()); // displaying the ticks for the arm.

        if (isTankPBot) {
            shuffleboardTab.add("Temp Right One", drivetrain.getTempRightOne());
            shuffleboardTab.add("Temp Right Two", drivetrain.getTempRightTwo());
            shuffleboardTab.add("Temp Left One", drivetrain.getTempLeftOne());
            shuffleboardTab.add("Temp Left Two", drivetrain.getTempLeftTwo());
        } // TODO: For swerve drivetrain rohan should take care of it.
    }
}
