package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShamLib.SMF.StateMachine;

import static frc.robot.subsystems.Lights.*;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import static frc.robot.subsystems.Lights.LEDState.*;
import static frc.robot.Constants.Lights.*;

public class Lights extends StateMachine<LEDState> {
    // private final CANdle candle = new CANdle(CANdleID, "rio");

    private Animation currentAnimation = lightsOff;

    public Lights() {
        super(LEDState.class);

        CANdleConfiguration config = new CANdleConfiguration();
        config.disableWhenLOS = true;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.1; //TODO: Raise, probably
        // config.vBatOutputMode = VBatOutputMode.Modulated;
        //TODO: Figure out what VBat is supposed to be
        // candle.configAllSettings(config, 100);

        addDetermination(Undetermined, Idle, setAnimation(idleAnimation));
        addCommutativeTransition(Idle, Testing, setAnimation(testingAnimation), setAnimation(lightsOff));
        addCommutativeTransition(Idle, Default, setAnimation(lightsOff), setAnimation(idleAnimation));
        addTransition(Idle, OneBall, setAnimation(oneBallAnimation));
        addTransition(Idle, TwoBall, setAnimation(twoBallAnimation));
        addCommutativeTransition(Default, BottomEject, setAnimation(bottomEjectAnimation), setAnimation(lightsOff));
        addCommutativeTransition(Default, OneBall, setAnimation(oneBallAnimation), setAnimation(lightsOff));
        addCommutativeTransition(OneBall, TwoBall, setAnimation(twoBallAnimation), setAnimation(oneBallAnimation));
        addCommutativeTransition(OneBall, LockedIn, setAnimation(lockedInOneBall), setAnimation(oneBallAnimation));
        addCommutativeTransition(TwoBall, LockedIn, setAnimation(lockedInTwoBall), setAnimation(twoBallAnimation));
        addTransition(LockedIn, BottomEject, setAnimation(bottomEjectAnimation));
        addTransition(OneBall, BottomEject, setAnimation(oneBallAnimation));
        addTransition(TwoBall, BottomEject, setAnimation(twoBallAnimation));

        addTransition(Idle, Climbing, setAnimation(climbingAnimation));
    }

    private Command setAnimation(Animation animation) {
        return new InstantCommand(() -> currentAnimation = animation);
    }   

    @Override
    public void update() {
        // candle.animate(currentAnimation);
    }

    @Override
    public String getName() {return "Lights";}

    @Override
    public void additionalSendableData(SendableBuilder builder) {}

    public enum LEDState {
        Undetermined, Idle, Default, OneBall, TwoBall, LockedIn, BottomEject, Climbing, Testing;

        private static LEDState[] vals = values();
        public LEDState next() { return vals[(this.ordinal() +1) % vals.length];}
    }
}
