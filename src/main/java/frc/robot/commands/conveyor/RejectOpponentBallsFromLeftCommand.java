package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.util.ballTracker.BallTracker;

import static frc.robot.subsystems.Conveyor.ConveyorState.*;
import static frc.robot.util.ballTracker.Ball.BallColorType.*;

public class RejectOpponentBallsFromLeftCommand extends CommandBase {
    private Conveyor conveyor;
    private BallTracker tracker;

    public RejectOpponentBallsFromLeftCommand(Conveyor conveyor, BallTracker ballTracker) {
        this.conveyor = conveyor;
        this.tracker = ballTracker;
    }

    @Override
    public boolean isFinished() {
        //When no more incorrect color balls are left in the robot, we are finished ejecting
        return tracker.getBallsOfColor(Opposing).size() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopRightConveyor();

        if(conveyor.shouldEndIntakeSequence()) {
            conveyor.goToStateCommandDelayed(Idle).schedule();
        } else {
            if(tracker.getNumberOfBalls() > 1) {
                conveyor.goToStateCommandDelayed(WaitForBallFromLeft).schedule();
            } else {
                conveyor.goToStateCommandDelayed(IntakeLeft).schedule();
            }
        }
    }
}
