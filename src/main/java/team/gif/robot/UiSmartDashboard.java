package team.gif.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team.gif.lib.autoMode;
import team.gif.lib.delay;
import team.gif.robot.commands.drivetrain.ResetHeading;

import static team.gif.robot.Robot.elevator;

public class UiSmartDashboard {

    public SendableChooser<autoMode> autoModeChooser = new SendableChooser<>();
    public SendableChooser<delay> delayChooser = new SendableChooser<>();

    /**
     *  Widgets (e.g. gyro),
     *  buttons (e.g. SmartDashboard.putData("Reset", new ResetHeading()); ),
     *  and Chooser options (e.g. auto mode)
     *
     *  Placed on a dashboard tab
     *  After SmartDashboard loads for the first time, items from network table onto Dashboard tab
     *  and save file as "YYYY shuffleboard layout.json"
     */
    public UiSmartDashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard"); // Gets a reference to the shuffleboard tab
        tab.add("BotHead", (x) -> {
                x.setSmartDashboardType("Gyro");
                x.addDoubleProperty("Value", () -> Robot.pigeon.getCompassHeading(), null);
            })
            .withPosition(5, 0);

        // Auto selections
        autoModeChooser.addOption("NONE", autoMode.NONE);
        autoModeChooser.setDefaultOption("P Cube High Engage", autoMode.PLACE_CUBE_HIGH_ENGAGE);
//        autoModeChooser.addOption("P Cone High Mobility", autoMode.PLACE_CONE_HIGH_MOBILITY);
//        autoModeChooser.addOption("P Cone Mid Mobility", autoMode.PLACE_CONE_MID_MOBILITY);
        autoModeChooser.addOption("P Cube High Mobility", autoMode.PLACE_CUBE_HIGH_MOBILITY);
//        autoModeChooser.addOption("P Cube Mid Mobility", autoMode.PLACE_CUBE_MID_MOBILITY);
        autoModeChooser.addOption("P Cube High Mobility Engage", autoMode.PLACE_MOBILITY_ENGAGE);
        autoModeChooser.addOption("P Cube High No Home Engage", autoMode.PLACE_CUBE_HIGH_NO_HOME_ENGAGE);
        autoModeChooser.addOption("P Collect Place", autoMode.PLACE_COLLECT_PLACE);

        tab.add("Auto Select", autoModeChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(7, 1)
            .withSize(2, 1);

        // Auto delay
        delayChooser.setDefaultOption("0", delay.DELAY_0);
        delayChooser.addOption("1", delay.DELAY_1);
        delayChooser.addOption("2", delay.DELAY_2);
        delayChooser.addOption("3", delay.DELAY_3);
        delayChooser.addOption("4", delay.DELAY_4);
        delayChooser.addOption("5", delay.DELAY_5);
        delayChooser.addOption("6", delay.DELAY_6);
        delayChooser.addOption("7", delay.DELAY_7);
        delayChooser.addOption("8", delay.DELAY_8);
        delayChooser.addOption("9", delay.DELAY_9);
        delayChooser.addOption("10", delay.DELAY_10);
        delayChooser.addOption("11", delay.DELAY_11);
        delayChooser.addOption("12", delay.DELAY_12);
        delayChooser.addOption("13", delay.DELAY_13);
        delayChooser.addOption("14", delay.DELAY_14);
        delayChooser.addOption("15", delay.DELAY_15);

        SmartDashboard.putData("Reset", new ResetHeading());

        tab.add("Delay", delayChooser)
            .withPosition(7, 0)
            .withSize(1, 1);

        SmartDashboard.putData("Elevator", new InstantCommand(elevator::zeroEncoder).ignoringDisable(true));
    }

    /**
     * Values which are updated periodically should be placed here
     *
     * Convenient way to format a number is to use putString w/ format:
     *     SmartDashboard.putString("Elevator", String.format("%11.2f", Elevator.getPosition()));
     */
    public void updateUI() {
        // Timers
        SmartDashboard.putString("Time", String.format("%.4f", Timer.getFPGATimestamp()));
        SmartDashboard.putString("Timer",String.format("%.2f",Timer.getMatchTime()));

        SmartDashboard.putBoolean("Cone/Cube", Robot.arm.getSensor());
    }
}
