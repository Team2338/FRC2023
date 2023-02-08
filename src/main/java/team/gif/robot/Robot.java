// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.lib.autoMode;
import team.gif.lib.delay;
import team.gif.robot.commands.drivetrain.DriveArcade;
import team.gif.robot.commands.drivetrain.DriveSwerve;
import team.gif.robot.commands.drivetrain.DriveTank;
import team.gif.robot.commands.arm.ArmManualControl;
import team.gif.robot.commands.elevator.ElevatorManualControl;
import team.gif.robot.subsystems.Arm;
import team.gif.robot.subsystems.Collector;
import team.gif.robot.subsystems.CollectorPneumatics;
import team.gif.robot.subsystems.Drivetrain;
import team.gif.robot.subsystems.Elevator;
import team.gif.robot.subsystems.SwerveDrivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    public static Drivetrain drivetrain;
    public static DriveTank tankDrive;
    public static DriveArcade arcadeDrive;
    public static SwerveDrivetrain swervetrain = null;
    public static DriveSwerve driveSwerve;
    private delay chosenDelay;
    private autoMode chosenAuto;
    private static final Timer elapsedTime = new Timer();
    private static boolean runAutoScheduler = false;
    private UiSmartDashboard uiSmartDashboard;

    public static Arm arm;
    public static Elevator elevator;
    public static Collector collector;
    public static CollectorPneumatics collectorPneumatics;
    public static OI oi;


    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        drivetrain = new Drivetrain(false, false);
        tankDrive = new DriveTank();
        arcadeDrive = new DriveArcade();
        swervetrain = new SwerveDrivetrain();
        driveSwerve = new DriveSwerve();
        swervetrain.resetHeading();
        arm = new Arm();
        elevator = new Elevator();
        collector = new Collector();
        collectorPneumatics = new CollectorPneumatics();
//        ui = new UI();
        uiSmartDashboard = new UiSmartDashboard();
        oi = new OI();

        if(isSwervePBot) {
            swervetrain.setDefaultCommand(driveSwerve);
        } else if (isTankPBot) {
            drivetrain.setDefaultCommand(arcadeDrive);
        }
        arm.setDefaultCommand(new ArmManualControl());
        elevator.setDefaultCommand(new ElevatorManualControl());
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
        elapsed.reset();
        elapsed.start();
        runAutoScheduler = true;

        autonomousCommand = robotContainer.getAutonomousCommand(chosenAuto);

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        if (runAutoScheduler && (elapsed.get() > (chosenDelay.getValue()))) {
            if (isSwervePBot) {
                autonomousCommand = robotContainer.getAutonomousCommand(autoMode.SWERVE_POC);
            } else {
                autonomousCommand = robotContainer.getAutonomousCommand(null);
            }
            if (autonomousCommand != null) {
                autonomousCommand.schedule();
            }
            runAutoScheduler = false;
            elapsed.stop();
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
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

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

    //TODO: Change and check before each usage
    public static boolean isCompBot = false;
    public static boolean isSwervePBot = false;
    public static boolean isTankPBot = true;
}
