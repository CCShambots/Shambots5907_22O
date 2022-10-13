package frc.robot.util.currentRegulator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class RegulatedCommand extends CommandBase {
    public abstract double[][] getCurrentPriorityList();
    public abstract double[] getCurrentUsage();
    public abstract double getCurrentUsageTotal();
    public abstract double[] getMinCurrents();
    public abstract double getMinCurrentTotal();

    public abstract void setCurrent(double current);
    public abstract void requestBrownOutCancel();

    public abstract int getPriority();

    public abstract void init();

    @Override
    public void initialize() {
        super.initialize();
        CurrentRegulator.getInstance().registerCommand(this);
        init();
    }
}
