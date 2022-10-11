package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.RobotManager;

import static frc.robot.util.RobotManager.RobotState.*;

public class FiveBall extends SequentialCommandGroup {

    public FiveBall(RobotManager rm) {
        addCommands(
                //TODO: Reset ball tracker with the one ball
                new ParallelCommandGroup(
                    rm.goToStateCommand(IntakeRight),
                    rm.trajectoryOnDt("five-ball-1", true)
                ),
                rm.autoTrackAndFire(),
                new ParallelCommandGroup(
                        rm.goToStateCommand(IntakeLeft),
                        rm.trajectoryOnDt("five-ball-2")
                ),
                rm.autoTrackAndFire(),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new WaitCommand(2),
                            rm.goToStateCommand(IntakeLeft)
                        )
                        rm.trajectoryOnDt("five-ball-3")
                ),
                rm.goToStateCommand(Idle),
                rm.autoTrackAndFire()
        );
    }
}
