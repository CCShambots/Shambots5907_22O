package frc.robot.util.currentRegulator;

import frc.robot.Constants;

import java.util.ArrayList;

public class CurrentRegulator {
    private static CurrentRegulator instance = null;
    private ArrayList<Regulatable> subsystems;

    private double maxCurrent = Constants.MAX_CURRENT;
    private boolean enabled = true;

    public CurrentRegulator() {
        subsystems = new ArrayList<>();
    }

    public static CurrentRegulator getInstance() {
        instance = instance == null ? new CurrentRegulator() : instance;
        return instance;
    }

    public void registerSubsystem(Regulatable subsystem) {
        subsystems.add(subsystem);
        updateSubsystemCurrentLimits();
    }

    public void updateSubsystemCurrentLimits() {
        limitTotalCurrent();

        double increaseFactor = maxCurrent/getTotalCurrentMin();

        for (Regulatable subsystem : subsystems) {
            subsystem.setCurrent(subsystem.getMinCurrentTotal() * increaseFactor);
        }
    }

    private void limitTotalCurrent() {
        while (getTotalCurrentMin() > maxCurrent) {
            Regulatable toCancel = getLeastImportantsubsystem();
            toCancel.suspend();
        }
    }

    private double getTotalCurrentMin() {
        double totalMin = 0;
        for (Regulatable subsystem : subsystems) {
            totalMin += subsystem.getMinCurrentTotal();
        }

        return totalMin;
    }

    private Regulatable getLeastImportantsubsystem() {
        Regulatable out = subsystems.get(0);

        for (Regulatable subsystem : subsystems) {
            if (subsystem.getPriority() < out.getPriority()) {
                out = subsystem;
            }
        }

        return out;
    }

    public void setLowPowerMode(boolean on) {
        maxCurrent = on ? Constants.MAX_CURRENT_LOW_POWER : Constants.MAX_CURRENT;
    }

    public boolean isLowPowerMode() {
        return maxCurrent == Constants.MAX_CURRENT_LOW_POWER;
    }

    public void disable() {
        enabled = false;
    }

    public void enable() {
        enabled = true;
    }

    public void update() {
        //TODO: log ig idk
    }
}