package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static com.ctre.phoenix.GadgeteerUartClient.GadgeteerProxyType.Pigeon;
import static team.gif.robot.Robot.arm;
import static team.gif.robot.Robot.elevator;

public class UI {

    public UI() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("FRC2023");

        shuffleboardTab.add("Arm Ticks", arm.getTicks()); // displaying the ticks for the arm.

        // Pigeon
        shuffleboardTab.add("Bot Heading", (x) -> {
            x.setSmartDashboardType("Gyro");
            x.addDoubleProperty("Value", () -> Robot.pigeon.getCompassHeading(), null);
        });
    }
}
