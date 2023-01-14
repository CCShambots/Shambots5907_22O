package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoEngageCommand extends CommandBase {

    private Drivetrain dt;

    public AutoEngageCommand(Drivetrain dt) {
        this.dt = dt;
    }

    @Override
    public void initialize() {
        dt.getGyroHeading();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
