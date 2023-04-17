package team.gif.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team.gif.lib.autoMode;
import team.gif.lib.delay;
import team.gif.robot.commands.drivetrain.Reset0;
import team.gif.robot.commands.drivetrain.Reset180;

import static team.gif.robot.Robot.elevator;

public class UiSmartDashboard {

    public SendableChooser<autoMode> autoModeChooser = new SendableChooser<>();
    public SendableChooser<delay> delayChooser = new SendableChooser<>();

    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autos");

    private ShuffleboardLayout UBIF = autoTab
            .getLayout("PCH E", BuiltInLayouts.kList)
            .withPosition(0,0)
            .withSize(1,2);

    public double FALLING_ANGLE;
    public GenericEntry fallingAngleUI = UBIF.add("Falling",-11.0)
            .withPosition(0,1)
            .getEntry();

    public double CROSSOVER_ANGLE;
    public GenericEntry crossoverAngleUI = UBIF.add("Crossover",-13.0)
            .withPosition(0,0)
            .getEntry();

    private ShuffleboardLayout UBIF_FFS = autoTab
            .getLayout("* ME", BuiltInLayouts.kList)
            .withPosition(1,0)
            .withSize(1,2);

    public double FFS_CROSSOVER_ANGLE;
    final public GenericEntry ffsCrossoverAngleUI = UBIF_FFS.add("FFS Crossover",13.0)
            .withPosition(0,0)
            .getEntry();

    public double FFS_FALLING_ANGLE;
    final public GenericEntry ffsFallingAngleUI = UBIF_FFS.add("FFS Falling",11.0)
            .withPosition(0,1)
            .getEntry();

    private ShuffleboardLayout UBIFRT = autoTab
            .getLayout("2GP CE", BuiltInLayouts.kList)
            .withPosition(2,0)
            .withSize(1,2);

    public double UBIFRT_CROSSOVER_ANGLE;
    final public GenericEntry ubifrtCrossoverAngleUI = UBIFRT.add("FFSRT Crossover",-13.0)
            .withPosition(0,0)
            .getEntry();

    public double UBIFRT_FALLING_ANGLE;
    final public GenericEntry ubifrtFallingAngleUI = UBIFRT.add("FFSRT Falling",-11.0)
            .withPosition(0,1)
            .getEntry();

    private ShuffleboardLayout UBIL = autoTab
            .getLayout("UBIL", BuiltInLayouts.kList)
            .withPosition(3,0)
            .withSize(1,2);

    public double THRESHOLD_ANGLE;
    public GenericEntry thresholdAngleUI = UBIL.add("Threshold <>+-",9.0)
            .withPosition(0,0)
            .getEntry();

    public double LEVEL_ANGLE;
    public GenericEntry levelAngleUI = UBIL.add("Level <",8.0)
            .withPosition(0,1)
            .getEntry();

    private ShuffleboardLayout collectorLayout = autoTab
            .getLayout("Collector", BuiltInLayouts.kList)
            .withPosition(4,0)
            .withSize(1,1);

    public double COLLECTOR_EJECT_SPEED;
    final public GenericEntry collectorEjectSpeedUI = collectorLayout.add("Speed",0.8)
            .getEntry();

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
        autoModeChooser.addOption("3 GP ^R Left", autoMode.THREE_GP_LEFT);
        autoModeChooser.setDefaultOption("3 GP ^R Right", autoMode.THREE_GP_RIGHT);
        autoModeChooser.addOption("2 GP ^R Center Engage", autoMode.PLACE_COLLECT_PLACE_ENGAGE_CENTER);
        autoModeChooser.addOption("2 GP ^ Left", autoMode.PLACE_COLLECT_PLACE_BARRIER);
        autoModeChooser.addOption("2 GP ^ Right", autoMode.PLACE_COLLECT_PLACE_CABLE);
        autoModeChooser.addOption("P Cube High Mobility Engage", autoMode.PLACE_CUBE_HIGH_MOBILITY_ENGAGE);
        autoModeChooser.addOption("P Cube High Engage", autoMode.PLACE_CUBE_HIGH_ENGAGE);
        autoModeChooser.addOption("P Cube High No Home Engage", autoMode.PLACE_CUBE_HIGH_NO_HOME_ENGAGE);
        autoModeChooser.addOption("P Cube High Mobility", autoMode.PLACE_CUBE_HIGH_MOBILITY);
        autoModeChooser.addOption("P ^ Mobility Engage Left", autoMode.PLACE_MOBILITY_ENGAGE_BARRIER);
        autoModeChooser.addOption("P ^ Mobility Engage Right", autoMode.PLACE_MOBILITY_ENGAGE_CABLE);

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

        SmartDashboard.putData("Reset", new Reset0());
        SmartDashboard.putData("Reset 180", new Reset180());

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
