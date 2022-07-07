package frc.robot.util;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;

public class RobotManager extends StatedSubsystem<RobotManager.State> {
    private boolean enabled = false;

    public RobotManager() {
        super(State.class);
    }

    @Override
    public void update() {

    }

    @Override
    public String getName() {
        return null;
    }


    public enum State {
        Undetermined, Idle, IntakeLeft, IntakeRight, EjectBottom, EjectTop, Shoot, Climb, Test
    }

    @Override
    public void additionalSendableData(SendableBuilder builder) {

    }
}
