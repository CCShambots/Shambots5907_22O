package frc.robot.util.hardware;

import edu.wpi.first.wpilibj.DigitalInput;

public class MagneticLimitSwitch {

    private DigitalInput pin;

    public MagneticLimitSwitch(int pin) {
        this.pin = new DigitalInput(pin);
    }

    //TODO: This perhaps should not be inverted (we'll wait for having the actual bot to test)
    public boolean isTripped() {
        return !pin.get();
    }

}
