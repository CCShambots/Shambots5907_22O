package frc.robot.util.hardware;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ColorSensor implements Sendable{
    private DigitalInput pin1;
    private DigitalInput pin2;

    public ColorSensor(int port1, int port2) {
        this.pin1 = new DigitalInput(port1);
        this.pin2 = new DigitalInput(port2);
    }

    public ColorSensorOutput getColor() {
        if(getPin1()) {
            if(getPin2()) return ColorSensorOutput.NoBallDetected;
            else return ColorSensorOutput.Red;
        } else {
            if(getPin2()) return ColorSensorOutput.Blue;
            else return ColorSensorOutput.SensorNotDetected;
        }

    }

    private boolean getPin1() {return pin1.get();} 
    private boolean getPin2() {return pin2.get();}

    /**
     * Create a new trigger to run when the state of the color sensor changes to 'value'
     * @param value When the prox sensor reaches this value, it will run the runnable
     * @param toRun What to run
     */
    public void registerTrigger(ColorSensorOutput value, Runnable toRun) {
        new Trigger(() -> this.getColor() == value).whenActive(toRun);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("digital-color-sensor");
        builder.addBooleanProperty("pin-1", this::getPin1, null);
        builder.addBooleanProperty("pin-2", this::getPin2, null);
        builder.addStringProperty("color-value", () -> getColor().name(), null);
    }

    public enum ColorSensorOutput {
        Red, Blue, SensorNotDetected, NoBallDetected
    }

}