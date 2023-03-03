package team.gif.lib.actions;

public interface Action {
    boolean isFinished();

    void update();

    void done();

    void start();
}
