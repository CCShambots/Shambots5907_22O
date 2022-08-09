package frc.robot.util.hardware;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ProximitySensor implements Sendable {
    private DigitalInput input;

    public ProximitySensor(int pin) {
        input = new DigitalInput(pin);
    }

    public boolean isActivated() {
        return !input.get();
    }

    /**
     * Create a new trigger to run when the state of the prox sensor changes to 'value'
     * @param value When the prox sensor reaches this value, it will run the runnable
     * @param toRun What to run
     */
    public void registerTrigger(boolean value, Runnable toRun) {
        new Trigger(() -> this.isActivated() == value).whenActive(toRun);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("prox-sensor");
        builder.addBooleanProperty("activated", () -> isActivated(), null);
    }
}
