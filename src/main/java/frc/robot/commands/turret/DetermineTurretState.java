package frc.robot.commands.turret;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class DetermineTurretState extends CommandBase {
    private double hoodPower = 0.125;
    private double rotaryPower = 0.125;
    private double outerRotaryResetAngle = 270;
    private double centerRotaryResetAngle = 270;

    private Turret t;

    private boolean resettingHood;
    private boolean resettingRotary;

    private double hoodStallVelocity = 5;//Minimum deg/sec before stall is counted

    private BooleanSupplier defaultTrust;

    public DetermineTurretState(Turret t, BooleanSupplier defaultTrust) {
        this.t = t;
        this.defaultTrust = defaultTrust;
    }

    @Override
    public void initialize() {

        resettingHood = true;
        resettingRotary = false;

        t.disableHoodControlLoops();
        t.disableRotaryControlLoops();

        t.setRotaryPower(rotaryPower);
        t.setHoodPower(-hoodPower);
    }

    @Override
    public void execute() {
        //Hood logic
        if(t.getHoodVelo() < hoodStallVelocity) {
            resettingHood = false;
            t.resetHoodPos(0);
        }

        if(t.isOuterLimPressed()) {
            resettingRotary = false;
            t.resetRotaryPos(outerRotaryResetAngle);
            t.setRotaryPower(0);
        } else if(t.isCenterLimPressed()) {
            resettingRotary = false;
            t.resetRotaryPos(centerRotaryResetAngle);
            t.setRotaryPower(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        t.enableHoodControlLoops();
        t.enableRotaryControlLoops();

        t.setHoodTargetAngle(0);
    }

    @Override
    public boolean isFinished() {
        return (!resettingHood && !resettingRotary ) || defaultTrust.getAsBoolean();
    }

}
