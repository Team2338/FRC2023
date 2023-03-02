package team.gif.lib;

public enum autoMode {
    SWERVE_POC(0),
    ENGAGE(0),
    PLACE_COLLECT(0),
    PLACE_CUBE_HIGH_ENGAGE(0),
    PLACE_CONE_HIGH_MOBILITY(0),
    PLACE_CONE_MID_MOBILITY(0),
    PLACE_CUBE_HIGH_MOBILITY(0),
    PLACE_CUBE_MID_MOBILITY(0),
    PLACE_MOBILITY_ENGAGE(0),
    PLACE_COLLECT_PLACE(0),
    ;

    private int value;

    autoMode(int value) {
        this.value = value;
    }

    public int getValue() {
        return this.value;
    }
}
