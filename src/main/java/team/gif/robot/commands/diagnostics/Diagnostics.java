package team.gif.robot.commands.diagnostics;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Globals;
import team.gif.robot.Robot;
import team.gif.robot.commands.collector.CollectorCollect;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.collector.WheelsOut;
import team.gif.robot.commands.combo.GoHome;
import team.gif.robot.commands.combo.GoLocation;

public class Diagnostics extends SequentialCommandGroup {

    //Elevator and Arm
    public static boolean elevatorAndArm = false; //Working or not
    public static String elevatorAndArmProblem; //state the problem

    //Collector
    public static boolean collector = false;
    public static String collectorProblem;

    public Diagnostics() {
        Globals.diagnosticsFlag = true; //setting the flag to true
        addCommands(
                new WaitCommand(2),
                new GoLocation(Constants.Location.LOAD_FROM_SINGLE_SUBSTATION), //going to single substation
                new WheelsOut(), //collector wheels out (cone pos)
                /**
                 * Collector will run it's own diagnostics if the diagnostics flag in true
                 * Just call CollectorCollect()
                 * **/
                new CollectorCollect().until(Robot.arm.armGamePieceSensor::get), //wait 1s and then run wheels until GP collected
                //new GoHome(), //go home pos // TODO: I don't think we need this.

                new WaitCommand(3).andThen(new GoLocation(Constants.Location.PLACE_CONE_HIGH)), //wait for 5s and go to place cone high pos
                new Check("Elevator And Arm", true),
                new WaitCommand(6).andThen(new WheelsIn()), //release the cone
                new GoHome(), //go home pos

                new WaitCommand(5)
        );
        Robot.oi.setRumbleDia(true);
    }

    public boolean getDiagnosticsRunning() {return Globals.diagnosticsFlag;}
    public boolean getArmAndEle() {return elevatorAndArm;}
    public String getElevatorAndArmProblem() {return elevatorAndArmProblem;}
    public boolean getCollector() {return collector;}
    public String getCollectorProblem() {return collectorProblem;}
}
