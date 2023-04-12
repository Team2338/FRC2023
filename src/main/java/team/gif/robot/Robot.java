// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import team.gif.lib.autoMode;
import team.gif.lib.delay;
import team.gif.robot.commands.arm.ArmPIDControl;
import team.gif.lib.logging.EventFileLogger;
import team.gif.lib.logging.TelemetryFileLogger;
import team.gif.robot.commands.collector.WheelsDefault;
import team.gif.robot.commands.drivetrain.DriveArcade;
import team.gif.robot.commands.drivetrain.DriveSwerve;
import team.gif.robot.commands.elevator.ElevatorPIDControl;
import team.gif.robot.commands.led.LEDSubsystemDefault;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.subsystems.Arm;
import team.gif.robot.subsystems.Collector;
import team.gif.robot.subsystems.CollectorWheels;
import team.gif.robot.subsystems.Drivetrain;
import team.gif.robot.subsystems.Elevator;
import team.gif.robot.subsystems.LEDSubsystem;
import team.gif.robot.subsystems.SwerveDrivetrain;
import team.gif.robot.subsystems.drivers.Pigeon;
import team.gif.robot.subsystems.drivers.Limelight;
import team.gif.robot.subsystems.TelescopingArm;
import team.gif.robot.subsystems.drivers.RobotCompressor;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static Command autonomousCommand;
    private RobotContainer robotContainer;
    private static autoMode chosenAuto;
    private static delay chosenDelay;
    private static TelemetryFileLogger telemetryLogger;
    public static EventFileLogger eventLogger;
    public static Drivetrain drivetrain;
    public static DriveArcade arcadeDrive;
    public static SwerveDrivetrain swervetrain = null;
    public static DriveSwerve driveSwerve;
    public static Limelight limelightHigh;
    public static Limelight limelightLow;
    public static Arm arm;
    public static Elevator elevator;
    public static Collector collector;
    public static CollectorWheels collectorWheels;
    public static TelescopingArm telescopingArm;
    public static OI oi;
    public static UiSmartDashboard uiSmartDashboard;
    public static LEDSubsystem ledSubsystem;
    public static RobotCompressor compressor;
    private Timer elapsedTime;
    private boolean runAutoScheduler;
    public static boolean runningAutonomousMode;

    public static Pigeon pigeon;

    public static UI ui;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        eventLogger = new EventFileLogger();
        eventLogger.init();

        telemetryLogger = new TelemetryFileLogger();
        addMetricsToLogger();

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        arm = new Arm();
        elevator = new Elevator();
        collector = new Collector();
        collectorWheels = new CollectorWheels();
        telescopingArm = new TelescopingArm();
        pigeon = isCompBot ? new Pigeon(RobotMap.PIGEON_COMP_PBOT) : new Pigeon(new TalonSRX(RobotMap.PIGEON_TANK_PBOT));
        limelightHigh = new Limelight();
        limelightLow = new Limelight("limelight-low");
        ledSubsystem = new LEDSubsystem();
        compressor = new RobotCompressor(RobotMap.COMPRESSOR, PneumaticsModuleType.REVPH);

        swervetrain = new SwerveDrivetrain(telemetryLogger);
        driveSwerve = new DriveSwerve();
        swervetrain.setDefaultCommand(driveSwerve);
        swervetrain.resetHeading();


        ui = new UI();
        uiSmartDashboard = new UiSmartDashboard();

        arm.setTargetPosition(arm.getPosition());
        arm.setDefaultCommand(new ArmPIDControl());

        ledSubsystem.setDefaultCommand(new LEDSubsystemDefault());

        elevator.setElevatorTargetPos(elevator.getPosition());
        elevator.setDefaultCommand(new ElevatorPIDControl());

        collectorWheels.setDefaultCommand(new WheelsDefault());

        // settings default wheels to WheelsIn;
        collectorWheels.wheelsIn();

        limelightLow.setLEDOff();
        limelightHigh.setLEDOff();

        oi = new OI();

        SmartDashboard.putNumber("Auto Time",Constants.AutoConstants.DRIVE_TIME_DEFAULT);
        // TODO SwerveAuto remove after PID constants are finalized and autos are running well
        //SA SmartDashboard.putNumber("kPX", 5.0);
        //SA SmartDashboard.putNumber("kPY", 5.0);
        //SA SmartDashboard.putNumber("kPTheta", 3.7);

        if (isCompBot) {
//SB            ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

        }

        elapsedTime = new Timer();
        telemetryLogger.init();
        // TODO SwerveAuto put back after PID constants are finalized and autos are running well
        robotContainer = new RobotContainer();
        runningAutonomousMode = false;
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        uiSmartDashboard.updateUI();

        chosenAuto = uiSmartDashboard.autoModeChooser.getSelected();
        chosenDelay = uiSmartDashboard.delayChooser.getSelected();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        // TODO SwerveAuto remove after PID constants are finalized and autos are running well
        //robotContainer = new RobotContainer();

        runningAutonomousMode = true;

        eventLogger.addEvent("AUTO", "Auto Init");
        eventLogger.addEvent("AUTO", "Reset sensors");
//        pigeon.resetPigeonPosition(180);

        autonomousCommand = robotContainer.getAutonomousCommand(chosenAuto);
        eventLogger.addEvent("AUTO", "Got command from container");
        elapsedTime.reset();
        elapsedTime.start();

        runAutoScheduler = true;

        compressor.disable();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        collector.resetTimer();
        if (runAutoScheduler && (elapsedTime.get() > (chosenDelay.getValue()))) {
            if (autonomousCommand != null) {
                autonomousCommand.schedule();
            }
            runAutoScheduler = false;
            elapsedTime.stop();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        runningAutonomousMode = false;

        compressor.enableDigital();

        new ArmIn().schedule(); // at the ned of auto, arm may be out (e.g. shooting from charging station)
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double timeLeft = DriverStation.getMatchTime();
        oi.setRumble((timeLeft <= 40.0 && timeLeft >= 36.0) ||
                (timeLeft <= 25.0 && timeLeft >= 21.0) ||
                (timeLeft <= 5.0 && timeLeft >= 3.0));

        telemetryLogger.run();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    private void addMetricsToLogger() {
        telemetryLogger.addMetric("TimeStamp", Timer::getFPGATimestamp);

        telemetryLogger.addMetric("Driver_Left_Y", () -> -Robot.oi.driver.getLeftY());
        telemetryLogger.addMetric("Driver_Left_X", () -> Robot.oi.driver.getLeftX());
        telemetryLogger.addMetric("Driver_Angle", () -> Math.atan(-Robot.oi.driver.getLeftY() / Robot.oi.driver.getLeftX()));
        telemetryLogger.addMetric("Driver_Right_X", () -> Robot.oi.driver.getRightX());
    }

    //TODO: Change and check before each usage
    public static boolean isCompBot = true;

    public static void cancelAuto() {
        autonomousCommand.cancel();
    }
}
