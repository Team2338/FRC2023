package team.gif.lib.motion;

import static team.gif.lib.motion.MotionUtil.kEpsilon;
import static team.gif.lib.util.Util.epsilonEquals;

public class MotionSegment {
    protected MotionState start;
    protected MotionState finish;

    public boolean isValid() {
        if (!epsilonEquals(start().acc(), end().acc(), kEpsilon)) {
            // Acceleration is not constant within the segment.
            System.err.println(
                    "Segment acceleration not constant! Start acc: " + start().acc() + ", End acc: " + end().acc());
            return false;
        }
        if (Math.signum(start().vel()) * Math.signum(end().vel()) < 0.0 && !epsilonEquals(start().vel(), 0.0, kEpsilon)
                && !epsilonEquals(end().vel(), 0.0, kEpsilon)) {
            // Velocity direction reverses within the segment.
            System.err.println("Segment velocity reverses! Start vel: " + start().vel() + ", End vel: " + end().vel());
            return false;
        }
        if (!start().extrapolate(end().t()).equals(end())) {
            // A single segment is not consistent.
            if (start().t() == end().t() && Double.isInfinite(start().acc())) {
                // One allowed exception: If acc is infinite and dt is zero.
                return true;
            }
            System.err.println("Segment not consistent! Start: " + start() + ", End: " + end());
            return false;
        }
        return true;
    }

    public boolean containsTime(double t) {
        return t >= start().t() && t <= end().t();
    }

    public boolean containsPos(double pos) {
        return pos >= start().pos() && pos <= end().pos() || pos <= start().pos() && pos >= end().pos();
    }

    public MotionState start() {
        return start;
    }

    public void setStart(MotionState start) {
        this.start = start;
    }

    public MotionState end() {
        return finish;
    }

    public void setEnd(MotionState end) {
        finish = end;
    }

    @Override
    public String toString() {
        return "Start: " + start() + ", End: " + end();
    }

    public MotionSegment(MotionState start, MotionState finish) {
        this.start = start;
        this.finish = finish;
    }
}
