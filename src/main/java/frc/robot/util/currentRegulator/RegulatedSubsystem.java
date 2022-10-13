package frc.robot.util.currentRegulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;

public abstract class RegulatedSubsystem<E extends Enum<E>> extends StatedSubsystem<E> {
    private E suspendedState;

    public RegulatedSubsystem(Class<E> enumType) {
        super(enumType);
    }

    public abstract double getCurrentUsageTotal();
    public abstract double getMinCurrentTotal();
    public abstract int getCurrentPriority();

    public abstract void setCurrent(double current);
    public void suspend() {
        requestTransition(suspendedState);
    }

    public boolean isSuspended() {
        return isInState(suspendedState);
    }
}
