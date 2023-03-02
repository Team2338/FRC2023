package team.gif.lib.motion;

public class MotionProfileConstraints {
    protected double maxAbsVel = Double.POSITIVE_INFINITY;
    protected double maxAbsAcc = Double.POSITIVE_INFINITY;

    public MotionProfileConstraints(double max_vel, double max_acc) {
        this.maxAbsAcc = Math.abs(max_acc);
        this.maxAbsVel = Math.abs(max_vel);
    }

    public double getMaxAbsVel() {
        return maxAbsVel;
    }

    public double getMaxAbsAcc() {
        return maxAbsAcc;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof MotionProfileConstraints)) {
            return false;
        }
        final MotionProfileConstraints other = (MotionProfileConstraints) obj;
        return (other.getMaxAbsAcc() == getMaxAbsAcc()) && (other.getMaxAbsVel() == getMaxAbsVel());
    }
}
