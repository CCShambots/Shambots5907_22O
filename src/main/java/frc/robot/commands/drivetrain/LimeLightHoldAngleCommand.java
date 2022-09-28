package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.hardware.Limelight;

public class LimeLightHoldAngleCommand extends CommandBase {
    private Drivetrain drivetrain;
    private Limelight limelight;

    public LimeLightHoldAngleCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        limelight = Limelight.getInstance();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            // if (limelight.getXOffset().getDegrees() >= 1.0) {
                drivetrain.setHoldAngle(
                        drivetrain.getCurrentAngle().plus(limelight.getXOffset())
                );
            // }
        }
        else {
            Pose2d robotPose = drivetrain.getPose();
            drivetrain.setHoldAngle(
                    new Rotation2d(Math.atan2(
                            Constants.GOAL_POS.getY() - robotPose.getY(),
                            Constants.GOAL_POS.getX() - robotPose.getX()
                    ))
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
