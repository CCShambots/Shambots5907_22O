package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.RobotManager;

import static frc.robot.util.RobotManager.RobotState.*;

public class TwoBall extends SequentialCommandGroup {

    public TwoBall(RobotManager rm) {
        addCommands(
                rm.resetTrackerForAuto(),
                new ParallelCommandGroup(
                    rm.goToStateCommand(IntakeRight),
                    rm.trajectoryOnDt("two-ball", true)
                ),
                rm.finishIntaking(1, 2),
                rm.forceTrackAndFire()
        );
    }
}
