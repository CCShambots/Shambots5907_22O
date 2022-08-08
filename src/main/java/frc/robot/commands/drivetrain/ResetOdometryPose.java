package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetOdometryPose extends CommandBase{
    

    private Drivetrain drivetrain;
    private Pose2d pose;

    public ResetOdometryPose(Drivetrain dt, Pose2d pose) {
        this.drivetrain = dt;
        this.pose = pose;
    }

    public ResetOdometryPose(Drivetrain dt, Trajectory trajectory) {this(dt, trajectory.getInitialPose());}

    @Override
    public void initialize() {
        drivetrain.resetOdometryPose(pose);
    }

    @Override
    public boolean isFinished() {return true;}
}
