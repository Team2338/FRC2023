package team.gif.lib.motion;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import static team.gif.lib.motion.MotionUtil.kEpsilon;

public class MotionProfile {
    protected List<MotionSegment> segments;

    public MotionProfile() {
        segments = new ArrayList<>();
    }

    public MotionProfile(List<MotionSegment> motionSegments) {
        segments = motionSegments;
    }

    public boolean isValid() {
        MotionSegment prev_segment = null;
        for (MotionSegment s : segments) {
            if (!s.isValid()) {
                return false;
            }
            if (prev_segment != null && !s.start().coincident(prev_segment.end())) {
                // Adjacent segments are not continuous.
                System.err.println("Segments not continuous! End: " + prev_segment.end() + ", Start: " + s.start());
                return false;
            }
            prev_segment = s;
        }
        return true;
    }

    public boolean isEmpty() {
        return segments.isEmpty();
    }

    public List<MotionSegment> segments() {
        return segments;
    }

    public Optional<MotionState> stateByTime(double t) {
        if (t < startTime() && t + kEpsilon >= startTime()) {
            return Optional.of(startState());
        }
        if (t > endTime() && t - kEpsilon <= endTime()) {
            return Optional.of(endState());
        }
        for (MotionSegment s : segments) {
            if (s.containsTime(t)) {
                return Optional.of(s.start().extrapolate(t));
            }
        }
        return Optional.empty();
    }

    public MotionState startState() {
        if (isEmpty()) {
            return MotionState.kInvalidState;
        }
        return segments.get(0).start();
    }

    public double startPos() {
        return startState().pos();
    }

    public MotionState endState() {
        if (isEmpty()) {
            return MotionState.kInvalidState;
        }
        return segments.get(segments.size() - 1).end();
    }

    public double startTime() {
        return startState().t();
    }

    public MotionState stateByTimeClamped(double t) {
        if (t < startTime()) {
            return startState();
        } else if (t > endTime()) {
            return endState();
        }
        for (MotionSegment s : segments) {
            if (s.containsTime(t)) {
                return s.start().extrapolate(t);
            }
        }
        // Should never get here.
        return MotionState.kInvalidState;
    }

    public int size() {
        return segments.size();
    }

    public void appendControl(double acc, double dt) {
        if (isEmpty()) {
            System.err.println("Error!  Trying to append to empty profile");
            return;
        }
        MotionState last_end_state = segments.get(segments.size() - 1).end();
        MotionState new_start_state = new MotionState(last_end_state.t(), last_end_state.pos(), last_end_state.vel(),
                acc);
        appendSegment(new MotionSegment(new_start_state, new_start_state.extrapolate(new_start_state.t() + dt)));
    }

    public void appendSegment(MotionSegment segment) {
        segments.add(segment);
    }

    public void appendProfile(MotionProfile profile) {
        for (MotionSegment s : profile.segments()) {
            appendSegment(s);
        }
    }

    public double endTime() {
        return endState().t();
    }

    public double endPos() {
        return endState().pos();
    }

    public double duration() {
        return endTime() - startTime();
    }

    public double length() {
        double length = 0.0;
        for (MotionSegment s : segments()) {
            length += Math.abs(s.end().pos() - s.start().pos());
        }
        return length;
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder("Profile:");
        for (MotionSegment s : segments()) {
            builder.append("\n\t");
            builder.append(s);
        }
        return builder.toString();
    }

    public void consolidate() {
        for (Iterator<MotionSegment> iterator = segments.iterator(); iterator.hasNext() && segments.size() > 1; ) {
            MotionSegment s = iterator.next();
            if (s.start().coincident(s.end())) {
                iterator.remove();
            }
        }
    }

    public void reset(MotionState initial_state) {
        clear();
        segments.add(new MotionSegment(initial_state, initial_state));
    }

    public void clear() {
        segments.clear();
    }
}
