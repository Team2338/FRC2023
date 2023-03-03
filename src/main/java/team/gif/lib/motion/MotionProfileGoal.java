package team.gif.lib.motion;

import static team.gif.lib.util.Util.epsilonEquals;

public class MotionProfileGoal {
    public enum CompletionBehavior {
        OVERSHOOT,
        VIOLATE_MAX_ACCEL,
        VIOLATE_MAX_ABS_VEL
    }

    protected double pos;
    protected double maxAbsVel;
    protected CompletionBehavior completionBehavior = CompletionBehavior.OVERSHOOT;
    protected double posTolerance = 1E-3;
    protected double velTolerance = 1E-2;

    public MotionProfileGoal() {}

    protected void compatCheck() {
        if (maxAbsVel > velTolerance && completionBehavior == CompletionBehavior.OVERSHOOT) {
            completionBehavior = CompletionBehavior.VIOLATE_MAX_ACCEL;
        }
    }

    public MotionProfileGoal(double pos) {
        this.pos = pos;
        this.maxAbsVel = 0.0;
        compatCheck();
    }

    public MotionProfileGoal(double pos, double maxAbsVel) {
        this.pos = pos;
        this.maxAbsVel = maxAbsVel;
        compatCheck();
    }

    public double pos() {
        return pos;
    }

    public MotionProfileGoal(double pos, double maxAbsVel, CompletionBehavior completionBehavior) {
        this.pos = pos;
        this.maxAbsVel = maxAbsVel;
        this.completionBehavior = completionBehavior;
        compatCheck();
    }

    public MotionProfileGoal(double pos, double maxAbsVel, CompletionBehavior completionBehavior, double posTolerance, double velTolerance) {
        this.pos = pos;
        this.maxAbsVel = maxAbsVel;
        this.completionBehavior = completionBehavior;
        this.posTolerance = posTolerance;
        this.velTolerance = velTolerance;
        compatCheck();
    }

    public MotionProfileGoal(MotionProfileGoal other) {
        this(other.pos, other.maxAbsVel, other.completionBehavior, other.posTolerance, other.velTolerance);
    }

    public MotionProfileGoal flipped() {
        return new MotionProfileGoal(-pos, maxAbsVel, completionBehavior, posTolerance, velTolerance);
    }

    public double getPos() {
        return pos;
    }

    public CompletionBehavior getCompletionBehavior() {
        return completionBehavior;
    }

    public double getPosTolerance() {
        return posTolerance;
    }

    public boolean atGoalState(MotionState state) {
        return atGoalPos(state.pos()) && (Math.abs(state.vel()) < (maxAbsVel + velTolerance)
                || completionBehavior == CompletionBehavior.VIOLATE_MAX_ABS_VEL);
    }

    public boolean atGoalPos(double pos) {
        return epsilonEquals(pos, this.pos, posTolerance);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof MotionProfileGoal)) {
            return false;
        }
        final MotionProfileGoal other = (MotionProfileGoal) obj;
        return (other.getCompletionBehavior() == getCompletionBehavior()) && (other.getPos() == getPos())
                && (other.getMaxAbsVel() == getMaxAbsVel()) && (other.getPosTolerance() == getPosTolerance())
                && (other.getVelTolerance() == getVelTolerance());
    }

    public CompletionBehavior completion_behavior() {
        return completionBehavior;
    }

    public double getMaxAbsVel() {
        return maxAbsVel;
    }

    public double getVelTolerance() {
        return velTolerance;
    }
}
