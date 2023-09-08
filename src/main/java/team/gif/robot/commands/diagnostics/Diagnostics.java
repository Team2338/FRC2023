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
                new WaitCommand(2), //wait for 2s
                new Check("Collector", true), // checking the collector only
                new WaitCommand(3).andThen(new Check("Elevator And Arm", true)), //checking the elevator and arm only
                //new GoHome(), //go home pos TODO: to be tested if it goes to home when the

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
