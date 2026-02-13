package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class AxisMappingTable extends InterpolatingDoubleTreeMap{
    @Override
    public Double get(Double key) {
        if(key < 0.0){
            return Math.copySign(super.get(-key), key);
        }
        else{
            return super.get(key);
        }
    }
}
