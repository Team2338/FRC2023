package team.gif.lib.actions;

import edu.wpi.first.wpilibj.Timer;

public class WaitAction implements Action {
    private double timeToWait;
    private double startTime;

    public WaitAction(double timeToWait) {
        this.timeToWait = timeToWait;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - this.startTime >= timeToWait;
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }
}
