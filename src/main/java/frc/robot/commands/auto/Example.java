package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.RobotManager;

import java.util.Set;

public class Example extends SequentialCommandGroup {

    public Example(RobotManager robotManager) {
        addCommands(
            robotManager.trajectoryOnDt("example", true)
        );

    }
}
