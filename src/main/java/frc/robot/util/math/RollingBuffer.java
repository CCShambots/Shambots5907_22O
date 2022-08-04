package frc.robot.util.math;

import frc.robot.Robot;

import java.util.*;
import java.util.function.DoubleSupplier;

public class RollingBuffer {
    private List<Double> buffer = new ArrayList<>();
    private DoubleSupplier valueSupplier;
    private float updateRateMS;
    private int elementsToStore;

    public RollingBuffer(DoubleSupplier valueSupplier, float updateRateMS, float retentionTimeMS, Robot robot) {
        this.valueSupplier = valueSupplier;
        this.updateRateMS = updateRateMS;

        robot.addPeriodic(this::update, updateRateMS / 1000.0);

        elementsToStore = Math.round(retentionTimeMS/updateRateMS);
    }

    public void update() {
        buffer.add(valueSupplier.getAsDouble());

        if(buffer.size() > elementsToStore) {
            buffer.remove(elementsToStore);
        }
    }

    public double get(float msAgo) {
        return buffer.get((Math.round(msAgo/updateRateMS)));
    }

}
