package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.RobotManager;
import frc.robot.util.RobotManager.RobotState;

import static frc.robot.util.RobotManager.RobotState.*;

public class FiveBall extends SequentialCommandGroup {

    public FiveBall(RobotManager rm) {
        addCommands(
                rm.resetTrackerForAuto(),
                new ParallelCommandGroup(
                    rm.goToStateCommand(IntakeRight),
                    rm.trajectoryOnDt("five-ball-1", true)
                ),
                rm.finishIntaking(1, 2),
                rm.forceTrackAndFire(),
                new ParallelCommandGroup(
                    rm.goToStateCommand(IntakeLeft),
                    rm.trajectoryOnDt("five-ball-2")
                ),
                rm.finishIntaking(0, 1),
                rm.forceTrackAndFire(),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new WaitCommand(1),
                            rm.goToStateCommand(IntakeRight)
                        ),
                        rm.trajectoryOnDt("five-ball-3")
                ),
                new WaitCommand(1.25).raceWith(rm.waitForState(RobotState.Idle)),
                rm.trajectoryOnDt("five-ball-4"),
                rm.finishIntaking(0, 2),
                rm.forceTrackAndFire()
        );
    }
}
