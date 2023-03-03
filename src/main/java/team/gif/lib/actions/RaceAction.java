package team.gif.lib.actions;

public class RaceAction implements Action {
    private Action[] actions;

    private final Action leadAction;

    public RaceAction(Action leadAction, Action... actions) {
        this.actions = actions;
        this.leadAction = leadAction;
    }

    @Override
    public boolean isFinished() {
        if (leadAction.isFinished()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        leadAction.update();
        for (Action action: actions) {
            action.update();
        }
    }

    @Override
    public void done() {
        leadAction.done();
        for (Action action: actions) {
            action.done();
        }
    }

    @Override
    public void start() {
        leadAction.start();
        for (Action action : actions) {
            action.start();
        }
    }
}
