package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.RobotManager;

public class OneBall extends SequentialCommandGroup{

    public OneBall(RobotManager rm) {
        addCommands(
            rm.resetTrackerForAuto(),
            rm.forceTrackAndFire(),
            rm.trajectoryOnDt("one-ball", true)
        );
    }
    
}
