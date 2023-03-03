package team.gif.lib.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SeriesAction implements Action {
    private Action currentAction;
    private final ArrayList<Action> remainingActions;

    public SeriesAction(List<Action> actions) {
        remainingActions = new ArrayList<>(actions);
        currentAction = null;
    }

    public SeriesAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public void start() {}

    @Override
    public void update() {
        if (currentAction == null) {
            if (remainingActions.isEmpty()) {
                return;
            }

            currentAction = remainingActions.remove(0);
            currentAction.start();
        }

        currentAction.update();

        if (currentAction.isFinished()) {
            currentAction.done();
            currentAction = null;
        }
    }

    @Override
    public boolean isFinished() {
        return remainingActions.isEmpty() && currentAction == null;
    }

    @Override
    public void done() {}
}
