package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Globals;
import team.gif.robot.Robot;
import team.gif.robot.commands.diagnostics.Diagnostics;

public class CollectorCollect extends CommandBase {

    public CollectorCollect() {
        super();
        addRequirements(Robot.collector);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.collector.setSpeedPercentCollector(Constants.Collector.COLLECTOR_RUN_COLLECT);

        //Diagnostics details
        if (Globals.diagnosticsFlag) {
            if (Robot.collector.getSpeed() > Constants.Collector.COLLECTOR_RUN_COLLECT) {
                System.out.println("Collector Diagnostics");
                Diagnostics.collector = true;
                Diagnostics.collectorProblem = "No Problem";
            } else {
                Diagnostics.collector = false;
                Diagnostics.collectorProblem = "the collect wheels are not spinning at target speed, " + Robot.collector.getSpeed();
            }
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.collector.setSpeedPercentCollector(0);
    }
}
