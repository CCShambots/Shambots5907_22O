package frc.robot.util.currentRegulator;

public interface Regulatable {
    double getCurrentUsageTotal();
    double getMinCurrentTotal();
    int getCurrentPriority();
    void setCurrent(double current);
}
