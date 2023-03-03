package team.gif.lib.motion;

import java.util.Optional;

public class SetpointGenerator {
    public static class Setpoint {
        public MotionState motionState;
        public boolean finalSetpoint;

        public Setpoint(MotionState motion_state, boolean final_setpoint) {
            this.motionState = motion_state;
            this.finalSetpoint = final_setpoint;
        }
    }

    protected MotionProfile mProfile = null;
    protected MotionProfileGoal mGoal = null;
    protected MotionProfileConstraints mConstraints = null;

    public SetpointGenerator() {}

    /**
     * Force a reset of the profile.
     */
    public void reset() {
        mProfile = null;
        mGoal = null;
        mConstraints = null;
    }

    /**
     * Get a new Setpoint (and generate a new MotionProfile if necessary).
     *
     * @param constraints The constraints to use.
     * @param goal        The goal to use.
     * @param prev_state  The previous setpoint (or measured state of the system to do a reset).
     * @param t           The time to generate a setpoint for.
     * @return The new Setpoint at time t.
     */
    public synchronized Setpoint getSetpoint(MotionProfileConstraints constraints, MotionProfileGoal goal,
                                             MotionState prev_state,
                                             double t) {
        boolean regenerate = mConstraints == null || !mConstraints.equals(constraints) || mGoal == null
                || !mGoal.equals(goal) || mProfile == null;
        if (!regenerate && !mProfile.isEmpty()) {
            Optional<MotionState> expected_state = mProfile.stateByTime(prev_state.t());
            regenerate = !expected_state.isPresent() || !expected_state.get().equals(prev_state);
        }
        if (regenerate) {
            // Regenerate the profile, as our current profile does not satisfy the inputs.
            mConstraints = constraints;
            mGoal = goal;
            mProfile = MotionProfileGenerator.generateProfile(constraints, goal, prev_state);
            // System.out.println("Regenerating profile: " + mProfile);
        }

        // Sample the profile at time t.
        Setpoint rv = null;
        if (!mProfile.isEmpty() && mProfile.isValid()) {
            MotionState setpoint;
            if (t > mProfile.endTime()) {
                setpoint = mProfile.endState();
            } else if (t < mProfile.startTime()) {
                setpoint = mProfile.startState();
            } else {
                setpoint = mProfile.stateByTime(t).get();
            }
            // Shorten the profile and return the new setpoint.
            mProfile.trimBeforeTime(t);
            rv = new Setpoint(setpoint, mProfile.isEmpty() || mGoal.atGoalState(setpoint));
        }

        // Invalid or empty profile - just output the same state again.
        if (rv == null) {
            rv = new Setpoint(prev_state, true);
        }

        if (rv.finalSetpoint) {
            // Ensure the final setpoint matches the goal exactly.
            rv.motionState = new MotionState(rv.motionState.t(), mGoal.pos(),
                    Math.signum(rv.motionState.vel()) * Math.max(mGoal.getMaxAbsVel(), Math.abs(rv.motionState.vel())),
                    0.0);
        }

        return rv;
    }

    /**
     * Get the full profile from the latest call to getSetpoint(). Useful to check estimated time or distance to goal.
     *
     * @return The profile from the latest call to getSetpoint(), or null if there is not yet a profile.
     */
    public MotionProfile getProfile() {
        return mProfile;
    }
}
