package frc.robot.enums;

import java.util.HashMap;

public enum IntakeState {
    STOP(0),
    INTAKING(1),
    REVERSE(2);

    public final int value;

    IntakeState(int initValue) {
        this.value = initValue;
    }

    private static HashMap<Integer, IntakeState> _map = null;
    static {
        _map = new HashMap<Integer, IntakeState>();
        for (IntakeState type : IntakeState.values()) {
            _map.put(type.value, type);
        }
    }

    /**
     * Gets IntakeState from specified value
     *
     * @param value Value of IntakeState
     * @return IntakeState of specified value
     */
    public static IntakeState valueOf(int value) {
        IntakeState retval = _map.get(value);
        if (retval != null)
            return retval;
        return IntakeState.values()[0];
    }
}
