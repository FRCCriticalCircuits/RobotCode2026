package frc.robot.enums;

import java.util.HashMap;

public enum UpperStructureStates {
    IDLE(0),
    SHOOTING(1),
    REVERSE(2),
    INTAKING(3);

    public final int value;

    UpperStructureStates(int initValue) {
        this.value = initValue;
    }

    private static HashMap<Integer, UpperStructureStates> _map = null;
    static {
        _map = new HashMap<Integer, UpperStructureStates>();
        for (UpperStructureStates type : UpperStructureStates.values()) {
            _map.put(type.value, type);
        }
    }

    /**
     * Gets UpperStructureStates from specified value
     *
     * @param value Value of UpperStructureState
     * @return UpperStructureStates of specified value
     */
    public static UpperStructureStates valueOf(int value) {
        UpperStructureStates retval = _map.get(value);
        if (retval != null)
            return retval;
        return UpperStructureStates.values()[0];
    }
}
