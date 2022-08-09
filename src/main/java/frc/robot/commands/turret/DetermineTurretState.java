package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class DetermineTurretState extends CommandBase {
    private Turret t;

    private boolean resettingHood;
    private boolean resettingRotary;

    private double hoodStallVelocity = 5;//Minimum deg/sec before stall is counted

    public DetermineTurretState(Turret t) {
        this.t = t;
    }

    @Override
    public void initialize() {

        resettingHood = true;
        resettingRotary = false;

        t.disableHoodControlLoops();
        t.disableRotaryControlLoops();

        t.setRotaryPower(0.25);
        t.setHoodPower(-0.25);
    }

    @Override
    public void execute() {
        //Hood logic
        if(t.getHoodVelo() < 5) {
            resettingHood = false;
            t.resetHoodPos(0);
        }

        if(t.isSensor1Pressed()) {
            resettingRotary = false;
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
        return !resettingHood && !resettingRotary;
    }

}
