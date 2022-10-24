package frc.robot.util.hardware;

import edu.wpi.first.wpilibj.DigitalInput;

public class MagneticLimitSwitch {

    private DigitalInput pin;

    public MagneticLimitSwitch(int pin) {
        this.pin = new DigitalInput(pin);
    }

    public boolean isTripped() {
        return !pin.get();
    }
}
