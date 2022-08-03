package frc.robot.util.math;

import static java.lang.Math.*;

public class Range {
    private double min;
    private double max;

    public Range(double val1, double val2) {
        this.min = min(val1, val2);
        this.max = max(val1, val2);
    }

    public double getMin() {
        return min;
    }

    public double getMax() {
        return max;
    }

    public boolean isInRange(double value) {
        return value > min && value < max;
    }

    public double clipToRange(double val) {
        if(val > max) return max;
        else if(val < min) return min;
        else return val;
    }


}
