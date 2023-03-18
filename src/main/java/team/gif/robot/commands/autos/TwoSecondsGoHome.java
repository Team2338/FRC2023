package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.commands.combo.GoHome;

public class TwoSecondsGoHome extends CommandBase {
    private Timer elaspedTime = new Timer();
    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
        elaspedTime.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (elaspedTime.get() >= 2.0) {
            //elaspedTime.reset();
            new GoHome();
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
