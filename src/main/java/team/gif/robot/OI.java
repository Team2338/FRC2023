package team.gif.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team.gif.robot.commands.arm.ArmLift;
import team.gif.robot.commands.autoaim.LimeLightAutoAlign;
import team.gif.robot.commands.autos.DriveAndEngageCommand;
import team.gif.robot.commands.autos.DriveToChargingStationCommand;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.collector.CollectorCollect;
import team.gif.robot.commands.collector.ToggleWheelsInAndOut;
import team.gif.robot.commands.combo.GoHome;
import team.gif.robot.commands.combo.GoLocation;
import team.gif.robot.commands.combo.ToggleManualPIDControl;
import team.gif.robot.commands.drivetrain.MoveLeftSlow;
import team.gif.robot.commands.drivetrain.MoveRightSlow;
import team.gif.robot.commands.driveModes.EnableBoost;
import team.gif.robot.commands.led.ConeLED;
import team.gif.robot.commands.led.CubeLED;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.commands.telescopingArm.ArmOut;
import team.gif.robot.commands.telescopingArm.MoveArm;

public class OI {
    /*
     * Instantiate all joysticks/controllers and their buttons here
     *
     * Examples:
     * public final CommandXboxController driver = new CommandXboxController(0);
     *
     * public final Trigger dA = driver.a();
     */

    public final CommandXboxController driver = new CommandXboxController(RobotMap.DRIVER_CONTROLLER_ID);
    public final CommandXboxController aux = new CommandXboxController(RobotMap.AUX_CONTROLLER_ID);
    public final CommandXboxController test = new CommandXboxController(RobotMap.TEST_CONTROLLER_ID);

    public final Trigger dA = driver.a();
    public final Trigger dB = driver.b();
    public final Trigger dX = driver.x();
    public final Trigger dY = driver.y();
    public final Trigger dLBump = driver.leftBumper();
    public final Trigger dRBump = driver.rightBumper();
    public final Trigger dBack = driver.back();
    public final Trigger dStart = driver.start();
    public final Trigger dLStickBtn = driver.leftStick();
    public final Trigger dRStickBtn = driver.rightStick();
    public final Trigger dRTrigger = driver.rightTrigger();
    public final Trigger dLTrigger = driver.leftTrigger();
    public final Trigger dDPadUp = driver.povUp();
    public final Trigger dDPadRight = driver.povRight();
    public final Trigger dDPadDown = driver.povDown();
    public final Trigger dDPadLeft = driver.povLeft();

    public final Trigger aA = aux.a();
    public final Trigger aB = aux.b();
    public final Trigger aX = aux.x();
    public final Trigger aY = aux.y();
    public final Trigger aLBump = aux.leftBumper();
    public final Trigger aRBump = aux.rightBumper();
    public final Trigger aBack = aux.back();
    public final Trigger aStart = aux.start();
    public final Trigger aLStickBtn = aux.leftStick();
    public final Trigger aRStickBtn = aux.rightStick();
    public final Trigger aRTrigger = aux.rightTrigger();
    public final Trigger aLTrigger = aux.leftTrigger();
    public final Trigger aDPadUp = aux.povUp();
    public final Trigger aDPadRight = aux.povRight();
    public final Trigger aDPadDown = aux.povDown();
    public final Trigger aDPadLeft = aux.povLeft();

    public final Trigger tA = test.a();
    public final Trigger tB = test.b();
    public final Trigger tX = test.x();
    public final Trigger tY = test.y();
    public final Trigger tLBump = test.leftBumper();
    public final Trigger tRBump = test.rightBumper();
    public final Trigger tBack = test.back();
    public final Trigger tStart = test.start();
    public final Trigger tLStickBtn = test.leftStick();
    public final Trigger tRStickBtn = test.rightStick();
    public final Trigger tRTrigger = test.rightTrigger();
    public final Trigger tLTrigger = test.leftTrigger();
    public final Trigger tDPadUp = test.povUp();
    public final Trigger tDPadRight = test.povRight();
    public final Trigger tDPadDown = test.povDown();
    public final Trigger tDPadLeft = test.povLeft();

    public final Trigger gamePieceSensor = new Trigger(Robot.arm.armGamePieceSensor::get);

    public OI() {
    /*
     *
     * Create controller actions here
     *
     * Usages:
     * dRTrigger.whileTrue(new CollectCommand());
     * dLTrigger.onTrue(new EjectCommand());
     * dA.whileTrue(new RepeatCommand(new RapidFire());
     * aStart.onTrue(new InstantCommand(Robot.elevator::zeroEncoder).ignoringDisable(true));
     *
     * onTrue (fka whenPressed)    Init->Execute repeats until IsFinished = true->End, will not start again at Init if still held down
     * whileTrue (fka whenHeld)    Init->Execute repeats until IsFinished = true or button released->End, will not start again at Init if still held down
     * whileTrue(new RepeatCommand()) (fka whileHeld)   Init->Execute repeats until IsFinished = true or button released->End, will start again at Init if still held down
     *
     * Simple Test:
     *   aX.onTrue(new PrintCommand("aX"));
     */

        // elevator
        aStart.onTrue(new InstantCommand(Robot.elevator::zeroEncoder).ignoringDisable(true));

        aRTrigger.onTrue(new PrintCommand("aRTrigger"));

        // manual mode
        aBack.toggleOnTrue(new ToggleManualPIDControl());

        // combo loading actions
        aDPadUp.onTrue(new GoLocation(Constants.Location.LOAD_FROM_DOUBLE_SUBSTATION));
        aDPadRight.onTrue(new GoLocation(Constants.Location.LOAD_FROM_SINGLE_SUBSTATION));
        aDPadDown.onTrue(new GoLocation(Constants.Location.LOAD_FROM_FLOOR)); // GoFloor());
        aDPadLeft.onTrue(new GoHome());

        // combo placing cone actions
        aRBump.onTrue(new GoLocation(Constants.Location.PLACE_CONE_HIGH));
        aX.onTrue(new GoLocation(Constants.Location.PLACE_CONE_MID));

        // combo placing cube actions
        aY.onTrue(new GoLocation(Constants.Location.PLACE_CUBE_HIGH));
        aB.onTrue(new GoLocation(Constants.Location.PLACE_CUBE_MID));
        aA.onTrue(new GoLocation(Constants.Location.PLACE_LOW));

        // collector
        dRTrigger.whileTrue(new CollectorCollect());
        dLTrigger.whileTrue(new CollectorEject());

        dRBump.whileTrue(new CubeLED());
        dLBump.whileTrue(new ConeLED());

        dY.toggleOnTrue(new ToggleWheelsInAndOut());
        dB.onTrue(new LimeLightAutoAlign());
        dA.onTrue(new ArmLift());

        tX.whileTrue(new MoveArm(-0.2)); // goes in
        tY.whileTrue(new MoveArm(0.2)); // goes out
        tDPadRight.onTrue(new ArmOut(Constants.TelescopingArm.MAX_POS));
        tDPadLeft.onTrue(new ArmIn()); // move arm all in

        dLStickBtn.whileTrue(new EnableBoost());

        gamePieceSensor.onTrue(new InstantCommand(Robot.ledSubsystem::setLEDGamePieceColor));
        gamePieceSensor.onFalse(new InstantCommand(Robot.ledSubsystem::clearLEDGamePieceColor));
        // limelight toggle
//        dRTrigger.onTrue(new LedToggle());
        dBack.onTrue(new DriveAndEngageCommand());
        dStart.onTrue(new DriveToChargingStationCommand());

        dDPadLeft.whileTrue(new MoveLeftSlow());
        dDPadRight.whileTrue(new MoveRightSlow());
    }

    public void setRumble(boolean rumble) {
        driver.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, rumble ? 1.0 : 0.0);
        driver.getHID().setRumble(GenericHID.RumbleType.kRightRumble, rumble ? 1.0 : 0.0);
        aux.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, rumble ? 1.0 : 0.0);
        aux.getHID().setRumble(GenericHID.RumbleType.kRightRumble, rumble ? 1.0 : 0.0);
    }
}
