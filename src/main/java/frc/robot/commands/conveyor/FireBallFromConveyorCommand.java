package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.util.ballTracker.Ball;

import java.util.function.Supplier;

import static frc.robot.subsystems.Conveyor.ConveyorState.*;
import static frc.robot.util.ballTracker.Ball.BallPosition.*;

public class FireBallFromConveyorCommand extends CommandBase {
    private  Supplier<Ball> ballSupplier;
    private Conveyor conveyor;

    private Ball ballTracking;

    /**
     * Command that will continue running until the supplied ball has left the robot
     * @param ballSupplier A supplier that will be called on initialize()
     * @param conveyor
     */
    public FireBallFromConveyorCommand(Supplier<Ball> ballSupplier, Conveyor conveyor) {
        this.ballSupplier = ballSupplier;
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        ballTracking = ballSupplier.get();
    }

    @Override
    public boolean isFinished() {
        if(ballTracking == null) return true;
        else if(ballTracking.getPosition() == NotInBot) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.goToStateCommandDelayed(StartShooting).schedule();

        System.out.println("ENDING FIRE BALL FROM CONVEYOR COMMAND");
        if(interrupted) {
            conveyor.clearTracker();
            System.out.println("INTERRUPTED");
        }
    }
}
