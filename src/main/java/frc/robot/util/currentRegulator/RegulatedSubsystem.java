package frc.robot.util.currentRegulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;

public abstract class RegulatedSubsystem<E extends Enum<E>> extends StatedSubsystem<E> implements Regulatable {
    private E suspendedState;

    public RegulatedSubsystem(Class<E> enumType) {
        super(enumType);
    }

    public void suspend() {
        requestTransition(suspendedState);
    }

    public boolean isSuspended() {
        return isInState(suspendedState);
    }
}
