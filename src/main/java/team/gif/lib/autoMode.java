package team.gif.lib;

public enum autoMode {
    NONE(0),
    PLACE_CUBE_HIGH_ENGAGE(0),
    PLACE_CUBE_HIGH_MOBILITY(0),
    PLACE_CUBE_HIGH_MOBILITY_ENGAGE(0),
    PLACE_COLLECT_PLACE_CABLE(0),
    PLACE_COLLECT_PLACE_BARRIER(0),
    PLACE_CUBE_HIGH_NO_HOME_ENGAGE(0),
    PLACE_MOBILITY_ENGAGE_CABLE(0),
    PLACE_MOBILITY_ENGAGE_BARRIER(0),
    THREE_GP_RIGHT(0)
    ;

    private int value;

    autoMode(int value) {
        this.value = value;
    }

    public int getValue() {
        return this.value;
    }
}
