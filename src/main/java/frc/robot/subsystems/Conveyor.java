package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.conveyor.FireBallFromConveyorCommand;
import frc.robot.commands.conveyor.RejectOpponentBallsFromLeftCommand;
import frc.robot.commands.conveyor.RejectOpponentBallsFromRightCommand;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;
import frc.robot.util.ballTracker.Ball;
import frc.robot.util.ballTracker.BallTracker;
import frc.robot.util.ballTracker.Ball.BallColorType;
import frc.robot.util.hardware.ColorSensor;
import frc.robot.util.hardware.ProximitySensor;

import static frc.robot.Constants.Conveyor.*;
import static frc.robot.Constants.Turret.*;
import static frc.robot.subsystems.Conveyor.ConveyorState.*;
import static frc.robot.subsystems.Conveyor.MotorState.*;
import static frc.robot.util.ballTracker.Ball.BallColorType.*;
import static frc.robot.util.ballTracker.Ball.BallPosition.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class Conveyor extends StatedSubsystem<Conveyor.ConveyorState>{
    private final WPI_TalonFX leftCompactor = new WPI_TalonFX(LEFT_COMPACTOR_ID);
    private final WPI_TalonFX rightCompactor = new WPI_TalonFX(RIGHT_COMPACTOR_ID);
    private final WPI_TalonFX leftConveyor = new WPI_TalonFX(LEFT_CONVEYOR_ID);
    private final WPI_TalonFX rightConveyor = new WPI_TalonFX(RIGHT_CONVEYOR_ID);

    private final ColorSensor leftColorSensor = new ColorSensor(LEFT_COLOR_ONE, LEFT_COLOR_TWO);
    private final ColorSensor rightColorSensor = new ColorSensor(RIGHT_COLOR_ONE, RIGHT_COLOR_TWO);

    private final ProximitySensor leftProxSensor = new ProximitySensor(LEFT_PROX);
    private final ProximitySensor rightProxSensor = new ProximitySensor(RIGHT_PROX);
    private final ProximitySensor centerProxSensor = new ProximitySensor(CENTER_PROX);

    private final BallTracker ballTracker = new BallTracker(leftColorSensor, rightColorSensor, leftProxSensor, rightProxSensor, centerProxSensor, this);

    private boolean shouldEndIntakeSequence = false;

    public Conveyor() {
        super(ConveyorState.class);

        //TODO: Remove
        disableColorDetection();

        numBallsSupplier = () -> getNumBalls();

        Constants.configureMotor(leftCompactor, true, true, true);
        Constants.configureMotor(rightCompactor, true, true, true);
        Constants.configureMotor(leftConveyor);
        Constants.configureMotor(rightConveyor, true, true, true);

        Constants.defaultOptimizeMotor(leftCompactor);
        Constants.defaultOptimizeMotor(leftConveyor);
        Constants.defaultOptimizeMotor(rightCompactor);
        Constants.defaultOptimizeMotor(rightConveyor);

        addDetermination(Undetermined, Idle, new InstantCommand(this::stopAll));
        addTransition(Idle, StartIntakeLeft, new InstantCommand(() -> {
            setShouldEndIntakeSequence(false);
        }));

        setContinuousCommand(StartIntakeLeft, new InstantCommand(() -> {
            if(ballTracker.getNumberOfBalls() > 0 && ballTracker.findBall(PastRight) == null) {
                requestTransition(ShuffleToRight);
            } else {
                requestTransition(IntakeLeft);
            }
        }));

        addTransition(StartIntakeLeft, ShuffleToRight,
                new InstantCommand(() -> {exhaustRightConveyor(); exhaustRightCompactor(); intakeLeftConveyor(); intakeLeftCompactor();}));

        addTransition(StartIntakeLeft, IntakeLeft, new InstantCommand(() -> {intakeLeftConveyor(); intakeLeftCompactor(); }));

        setContinuousCommand(ShuffleToRight, new FunctionalCommand(() -> {}, () -> {}, (interrupted) -> {
            if(shouldEndIntakeSequence) {
                goToStateCommandDelayed(Idle).schedule();
            } else if(ballTracker.getNumberOfBalls() > 1) {
                goToStateCommandDelayed(WaitForBallFromLeft).schedule();
            } else if(ballTracker.getBallsOfColor(Opposing).size() > 0){
                goToStateCommandDelayed(EjectFromLeft).schedule();
            } else {
                goToStateCommandDelayed(IntakeLeft).schedule();
            }
        }, () -> ballTracker.isBallAtPos(PastRight)));

        addTransition(ShuffleToRight, Idle, new InstantCommand(() -> {stopAll(); setShouldEndIntakeSequence(false);}));
        addTransition(ShuffleToRight, IntakeLeft, new InstantCommand(() -> {stopRightConveyor(); stopRightCompactor();}));
        addTransition(ShuffleToRight, WaitForBallFromLeft, new InstantCommand(() -> {}));

        //When the second ball is acquired, reject it if it is the wrong color, end intaking if it is correct
        new Trigger(() -> this.getCurrentState() == IntakeLeft && ballTracker.getNumberOfBalls() > 1).whenActive(new InstantCommand(() -> {
            if(ballTracker.findBallSafe(Left).getColor() == Ours) {
                requestTransition(WaitForBallFromLeft);
            } else {
                requestTransition(EjectFromLeft);
            }
        }));

        //When the first ball is acquired, shuffle it if it is the right color, reject it if it is incorrect
        new Trigger(() -> this.getCurrentState() == IntakeLeft && ballTracker.getNumberOfBalls() == 1).whenActive(new InstantCommand(() -> {
            if(ballTracker.findBallSafe(Left).getColor() == Ours) {
                requestTransition(ShuffleToRight);
            } else if(ballTracker.findBallSafe(Left).getColor() == Opposing){
                requestTransition(EjectFromLeft);
            }
        }));

        addTransition(IntakeLeft, EjectFromLeft, new InstantCommand(() -> {intakeRightConveyor(); stopRightCompactor();}));
        addTransition(IntakeLeft, ShuffleToRight, new InstantCommand(() -> {exhaustRightConveyor(); exhaustRightCompactor();}));

        setContinuousCommand(EjectFromLeft, new RejectOpponentBallsFromLeftCommand(this, ballTracker));

        addTransition(EjectFromLeft, Idle, new InstantCommand(this::stopAll));
        addTransition(EjectFromLeft, WaitForBallFromLeft, new InstantCommand());
        addTransition(EjectFromLeft, IntakeLeft, new InstantCommand());

        addTransition(IntakeLeft, WaitForBallFromLeft, new InstantCommand());

        setContinuousCommand(WaitForBallFromLeft, new FunctionalCommand(() -> {}, () -> {},
                (interrupted) -> requestTransition(Idle), () -> ballTracker.isBallAtPos(Center)));

        addTransition(WaitForBallFromLeft, Idle, new InstantCommand(this::stopAll));

        setContinuousCommand(IntakeLeft, new FunctionalCommand(() ->{}, () -> {}, (interrupted) -> {if(shouldEndIntakeSequence) requestTransition(Idle);}, () -> shouldEndIntakeSequence));
        addTransition(IntakeLeft, Idle, new InstantCommand(() -> {stopAll(); setShouldEndIntakeSequence(false);}));




        /*
        Logic for intaking on the right side
         */




        addTransition(Idle, StartIntakeRight, new InstantCommand(() -> {
            setShouldEndIntakeSequence(false);
        }));

        setContinuousCommand(StartIntakeRight, new InstantCommand(() -> {
            if(ballTracker.getNumberOfBalls() > 0 && ballTracker.findBall(PastLeft) == null) {
                requestTransition(ShuffleToLeft);
            } else {
                requestTransition(IntakeRight);
            }
        }));

        addTransition(StartIntakeRight, ShuffleToLeft,
                new InstantCommand(() -> {exhaustLeftConveyor(); exhaustLeftCompactor(); intakeRightConveyor(); intakeRightCompactor();}));

        addTransition(StartIntakeRight, IntakeRight, new InstantCommand(() -> {intakeRightConveyor(); intakeRightCompactor(); }));

        setContinuousCommand(ShuffleToLeft, new FunctionalCommand(() -> {}, () -> {}, (interrupted) -> {
            if(shouldEndIntakeSequence) {
                goToStateCommandDelayed(Idle, 0.05).schedule();
            } else if(ballTracker.getNumberOfBalls() > 1) {
                goToStateCommandDelayed(WaitForBallFromRight, 0.05).schedule();
            } else if(ballTracker.getBallsOfColor(Opposing).size() > 0){
                goToStateCommandDelayed(EjectFromRight, 0.05).schedule();
            } else {
                goToStateCommandDelayed(IntakeRight, 0.05).schedule();
            }
        }, () -> ballTracker.isBallAtPos(PastLeft)));

        addTransition(ShuffleToLeft, Idle, new InstantCommand(() -> {stopAll(); setShouldEndIntakeSequence(false);}));
        addTransition(ShuffleToLeft, IntakeRight, new InstantCommand(() -> {stopLeftConveyor(); stopLeftCompactor();}));
        addTransition(ShuffleToLeft, WaitForBallFromRight, new InstantCommand(() -> {}));

        //When the second ball is acquired, reject it if it is the wrong color, end intaking if it is correct
        new Trigger(() -> this.getCurrentState() == IntakeRight && ballTracker.getNumberOfBalls() > 1).whenActive(new InstantCommand(() -> {
            if(ballTracker.findBallSafe(Right).getColor() == Ours) {
                requestTransition(WaitForBallFromRight);
            } else {
                requestTransition(EjectFromRight);
            }
        }));

        //When the first ball is acquired, shuffle it if it is the Left color, reject it if it is incorrect
        new Trigger(() -> this.getCurrentState() == IntakeRight && ballTracker.getNumberOfBalls() == 1).whenActive(new InstantCommand(() -> {
            if(ballTracker.findBallSafe(Right).getColor() == Ours) {
                requestTransition(ShuffleToLeft);
            } else if(ballTracker.findBallSafe(Right).getColor() == Opposing){
                requestTransition(EjectFromRight);
            }
        }));

        addTransition(IntakeRight, EjectFromRight, new InstantCommand(() -> {intakeLeftConveyor(); stopLeftCompactor();}));
        addTransition(IntakeRight, ShuffleToLeft, new InstantCommand(() -> {exhaustLeftConveyor(); exhaustLeftCompactor();}));

        setContinuousCommand(EjectFromRight, new RejectOpponentBallsFromRightCommand(this, ballTracker));

        addTransition(EjectFromRight, Idle, new InstantCommand(this::stopAll));
        addTransition(EjectFromRight, WaitForBallFromRight, new InstantCommand());
        addTransition(EjectFromRight, IntakeRight, new InstantCommand());

        addTransition(IntakeRight, WaitForBallFromRight, new InstantCommand());

        setContinuousCommand(WaitForBallFromRight, new FunctionalCommand(() -> {}, () -> {}, (interrupted) ->
                requestTransition(Idle), () -> ballTracker.isBallAtPos(Center)));

        addTransition(WaitForBallFromRight, Idle, new InstantCommand(this::stopAll));

        setContinuousCommand(IntakeRight, new FunctionalCommand(() ->{}, () -> {}, (interrupted) -> {if(shouldEndIntakeSequence) requestTransition(Idle);}, () -> shouldEndIntakeSequence));
        addTransition(IntakeRight, Idle, new InstantCommand(() -> {stopAll(); setShouldEndIntakeSequence(false);}));


        /*
        Logic for transitioning directly between intaking on each side
         */

        addTransition(IntakeLeft, StartIntakeRight, new InstantCommand(this::stopAll));
        addTransition(ShuffleToRight, StartIntakeRight, new InstantCommand(this::stopAll));
        addTransition(IntakeRight, StartIntakeLeft, new InstantCommand(this::stopAll));
        addTransition(ShuffleToLeft, StartIntakeLeft, new InstantCommand(this::stopAll));



        /*
        Logic for Shooting
         */

        Timer shootTimer = new Timer();

        addTransition(Idle, StartShooting, new InstantCommand(() -> {intakeLeftConveyor(); intakeRightConveyor(); shootTimer.reset(); shootTimer.start();}));
        setContinuousCommand(StartShooting, new InstantCommand(() -> {
            if(ballTracker.findBallsAtPositions(Left, BetweenLeftAndCenter, Center, BetweenRightAndCenter, Right).size() > 0) {
                requestTransition(ShootFromCenter);
            } else if(ballTracker.findBall(PastLeft) != null) {
                requestTransition(ShootFromLeft);
            } else if(ballTracker.findBall(PastRight) != null) {
                requestTransition(ShootFromRight);
            } else {
                requestTransition(Idle);
            }
        }));

        addTransition(StartShooting, ShootFromCenter, new InstantCommand());
        addTransition(StartShooting, ShootFromLeft, new InstantCommand(this::intakeLeftCompactor));
        addTransition(StartShooting, ShootFromRight, new InstantCommand(this::intakeRightCompactor));

        setContinuousCommand(ShootFromCenter, new FireBallFromConveyorCommand(
                () -> {
                    List<Ball> balls = ballTracker.findBallsAtPositions(Left, BetweenLeftAndCenter, Center, BetweenRightAndCenter, Right);
                    if(balls.size() > 0) return balls.get(0);
                    return null;
                }, this));

        setContinuousCommand(ShootFromLeft, new FireBallFromConveyorCommand(() -> ballTracker.findBall(PastLeft), this)
                .deadlineWith(new FunctionalCommand(() -> {}, () -> {}, (interrupted) -> {}, () -> shootTimer.get() > MAX_SHOOT_TIME)));
        setContinuousCommand(ShootFromRight, new FireBallFromConveyorCommand(() -> ballTracker.findBall(PastRight), this)
                .deadlineWith(new FunctionalCommand(() -> {}, () -> {}, (interrupted) -> {}, () -> shootTimer.get() > MAX_SHOOT_TIME)));

        addTransition(ShootFromCenter, StartShooting, new InstantCommand());
        addTransition(ShootFromLeft, StartShooting, new InstantCommand(this::stopLeftCompactor));
        addTransition(ShootFromRight, StartShooting, new InstantCommand(this::stopRightCompactor));

        addTransition(StartShooting, Idle, new InstantCommand(() -> {
            stopAll();
            shootTimer.stop();
        }));



        /*
        Logic for ejecting out the bottom
         */

        addOmniTransition(
                new ConveyorState[]{Idle, IntakeLeft, EjectFromLeft, ShuffleToLeft, StartIntakeLeft,
                        WaitForBallFromLeft, IntakeRight, EjectFromRight, ShuffleToRight,
                        StartIntakeRight, WaitForBallFromRight},
                new ConveyorState[]{EjectAll},
                new InstantCommand(this::exhaustAll));
        addTransition(EjectAll, Idle, new InstantCommand(() -> {
            stopAll();
            clearTracker();
        }));

    }

    public boolean shouldEndIntakeSequence() {
        return shouldEndIntakeSequence;
    }

    public void setShouldEndIntakeSequence(boolean shouldEndIntakeSequence) {
        this.shouldEndIntakeSequence = shouldEndIntakeSequence;
    }

    @Override
    public void onDisable() {
        runInstantaneousTransition(Idle,
                () -> {
                    stopAll();
                }
        );
    }

    @Override
    public void update() {
    }

    private void intakeLeftCompactor() {leftCompactor.set(COMPACTOR_SPEED);}
    private void intakeRightCompactor() {rightCompactor.set(COMPACTOR_SPEED);}
    private void intakeLeftConveyor() {leftConveyor.set(CONVEYOR_SPEED);}
    private void intakeRightConveyor() {rightConveyor.set(CONVEYOR_SPEED);}

    private void intakeAll() {intakeLeftCompactor(); intakeRightCompactor(); intakeLeftConveyor(); intakeRightConveyor();}

    private void exhaustLeftCompactor() {leftCompactor.set(-COMPACTOR_SPEED);}
    private void exhaustRightCompactor() {rightCompactor.set(-COMPACTOR_SPEED);}
    private void exhaustLeftConveyor() {leftConveyor.set(-CONVEYOR_SPEED);}
    private void exhaustRightConveyor() {rightConveyor.set(-CONVEYOR_SPEED);}

    private void exhaustAll() {exhaustLeftCompactor(); exhaustRightCompactor(); exhaustLeftConveyor(); exhaustRightConveyor();}

    private void stopLeftCompactor() {leftCompactor.set(0);}
    private void stopRightCompactor() {rightCompactor.set(0);}
    public void stopLeftConveyor() {leftConveyor.set(0);}
    public void stopRightConveyor() {rightConveyor.set(0);}

    private void stopAll() {stopLeftCompactor(); stopRightCompactor(); stopLeftConveyor(); stopRightConveyor();}

    public MotorState getLeftCompactorState() {return getMotorState(leftCompactor);}
    public MotorState getRightCompactorState() {return getMotorState(rightCompactor);}

    public MotorState getLeftConveyorState() {return getMotorState(leftConveyor);}
    public MotorState getRightConveyorState() {return getMotorState(rightConveyor);}


    /**
     * Get the state of a motor, where positive power is intaking and negative power is exhausting
     * @param motor the motor to evaluate
     * @return the state of the motor
     */
    private MotorState getMotorState(WPI_TalonFX motor) {
        if(motor.get() > 0) {
            return Intaking;
        } else if(motor.get() < 0) {
            return Exhausting;
        } else {
            return Stopped;
        }
    }

    public void resetBallTracker(List<Ball> newBallList) {
        ballTracker.resetTracker(newBallList);
    }

    public void clearTracker() {
        resetBallTracker(List.of(
                new Ball(BallColorType.NotInBot, Ball.BallPosition.NotInBot),
                new Ball(BallColorType.NotInBot, Ball.BallPosition.NotInBot),
                new Ball(BallColorType.NotInBot, Ball.BallPosition.NotInBot)
        ));
    }

    public int getNumBalls() {
        return ballTracker.getNumberOfBalls();
    }

    public void disableColorDetection() {ballTracker.disableColorDetection();}
    public void enableColorDetection() {ballTracker.enableColorDetection();}

    public enum ConveyorState {
        Undetermined, Idle,
        EjectAll, //Ejecting all balls from the bottom
        StartShooting, ShootFromCenter, ShootFromLeft, ShootFromRight, //States for shooting
        IntakeLeft, EjectFromLeft, ShuffleToRight, StartIntakeLeft, WaitForBallFromLeft, //States for intaking from the left side
        IntakeRight, EjectFromRight, ShuffleToLeft, StartIntakeRight, WaitForBallFromRight //States for intaking from the right side
    }

    public enum MotorState {
        Intaking, Exhausting, Stopped
    }

    @Override
    public String getName() {return "conveyor";}

    @Override
    public void additionalSendableData(SendableBuilder builder) {
        // builder.addDoubleProperty("left-conveyor-speed", leftConveyor::get, null);
        // builder.addDoubleProperty("right-conveyor-speed", rightConveyor::get, null);
        // builder.addDoubleProperty("left-compactor-speed", leftCompactor::get, null);
        // builder.addDoubleProperty("right-compactor-speed", rightCompactor::get, null);
        // builder.addBooleanProperty("should-end-intake-sequence", this::shouldEndIntakeSequence, null);
        // builder.addBooleanProperty("color-detection-enabled", ballTracker::isColorDetectionEnabled, null);
        builder.addDoubleProperty("ball-count", () -> ballTracker.getNumberOfBalls(), null);
    }

    @Override
    public Map<String, Sendable> additionalSendables() {
        return Map.of(
                "left-color", leftColorSensor,
                "right-color", rightColorSensor,
                "left-prox", leftProxSensor,
                "right-prox", rightProxSensor,
                "center-prox", centerProxSensor,
                "ball-1", ballTracker.getBall1(),
                "ball-2", ballTracker.getBall2(),
                "ball-3", ballTracker.getBall3()
        );
    }
}
