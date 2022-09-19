package frc.robot.util;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.util.Shambots5907_SMF.SimpleTransition;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

import static frc.robot.util.RobotManager.RobotState.*;

public class RobotManager extends StatedSubsystem<RobotManager.RobotState> {
    private Drivetrain dt;
    private Intake i;
    private Conveyor co;
    private Turret t;
    private Climber c;
    private Lights l;

    private Map<SimpleTransition<RobotState>, List<BooleanSupplier>> safeConditions = new HashMap<>();

    private int prevBallCount;
    private boolean prevShouldDisplayBallCount;
    private Constants.RobotEnabled prevBotEnabledStatus;

    public RobotManager(Drivetrain dt, Intake i, Conveyor co, Turret t, Climber c, Lights l) {
        super(RobotState.class);

        this.dt = dt;
        this.i = i;
        this.co = co;
        this.t = t;
        this.c = c;
        this.l = l;

        //Determination Logic; this transition will start instantly upon boot of robot code, but won't finish until every subsystem is enabled
        addDetermination(Undetermined, Idle, new ParallelCommandGroup(
                dt.waitForState(dt.getEntryState()),
                i.waitForState(i.getEntryState()),
                co.waitForState(co.getEntryState()),
                t.waitForState(t.getEntryState()),
                c.waitForState(c.getEntryState()),
                l.waitForState(l.getEntryState())
        ));

        //No conditions for undetermined --> idle; always return true
        addSafeTransitionCondition(Undetermined, Idle, () -> true);

        //Running command for idle
        setContinuousCommand(Idle, new ParallelCommandGroup(
                new RunCommand(() -> {
                    //Always set the turret back to active tracking while in idle (but not in a way that constantly cancels a transition)
                    if(!t.isInState(Turret.TurretState.ActiveTracking) && !t.isTransitioning()) {
                        t.requestTransition(Turret.TurretState.ActiveTracking);
                    }
                }),
                getLightControlCommand() //Run light control for displaying stuff while in Idle mode
        ));

        //Test logic
        addTransition(Idle, Test, new InstantCommand(() -> {
            c.requestTransition(Climber.ClimberState.Test);
            l.requestTransition(Lights.LEDState.Testing);
        }));
        
        //Since the bot would already be idle, there is no need for checks when moving to test state
        addSafeTransitionCondition(Idle, Test, () -> true);

        //Manual bottom eject logic
        addTransition(Idle, EjectBottom, new InstantCommand(() -> {
            i.requestTransition(Intake.IntakeState.Ejecting);
            co.requestTransition(Conveyor.ConveyorState.EjectAll);
            l.requestTransition(Lights.LEDState.BottomEject);
        }));
    
        //The bot should only do manual ejection if conveyor and intake are idling
        addSafeTransitionCondition(Idle, EjectBottom, () -> co.isInState(Conveyor.ConveyorState.Idle), () -> i.isInState(Intake.IntakeState.Idle));

        setContinuousCommand(EjectBottom, new WaitCommand(Constants.Intake.MANUAL_EJECT_TIME).andThen(new InstantCommand(() -> requestTransition(Idle))));

        addTransition(EjectBottom, Idle, new InstantCommand(() -> {
            i.requestTransition(Intake.IntakeState.Idle);
            co.requestTransition(Conveyor.ConveyorState.Idle);
            l.requestTransition(Lights.LEDState.Default);
        }));

        //Intake Cycle logic
        addTransition(Idle, IntakeRight, new InstantCommand(() -> {
            i.requestTransition(Intake.IntakeState.RightSideRunning);
            co.requestTransition(Conveyor.ConveyorState.StartIntakeRight);
        }));

        //The bot should only be allowed to start intaking if the conveyor and intake are idling (i.e. not still running from "eject bottom")
        addSafeTransitionCondition(Idle, IntakeRight, () -> co.isInState(Conveyor.ConveyorState.Idle), () -> i.isInState(Intake.IntakeState.Idle));

        addTransition(IntakeRight, Idle, new InstantCommand(() -> {
            i.requestTransition(Intake.IntakeState.Idle);
            co.setShouldEndIntakeSequence(true); //We shouldn't directly request Idle because the intaking subroutine occurs with the shouldEndIntakeSequence flag

            if(t.isInState(Turret.TurretState.ActiveTracking) && !t.isTransitioning()) {
                t.requestTransition(Turret.TurretState.ActiveTracking);
            }
        }));

        //If we're already intaking right, it's safe to go back to idle
        addSafeTransitionCondition(IntakeLeft, Idle, () -> i.isInState(Intake.IntakeState.RightSideRunning));

        addTransition(Idle, IntakeLeft, new InstantCommand(() -> {
            i.requestTransition(Intake.IntakeState.LeftSideRunning);
            co.requestTransition(Conveyor.ConveyorState.StartIntakeLeft);
        }));

        //The bot should only be allowed to start intaking if the conveyor and intake are idling (i.e. not still running from "eject bottom")
        addSafeTransitionCondition(Idle, IntakeRight, () -> co.isInState(Conveyor.ConveyorState.Idle), () -> i.isInState(Intake.IntakeState.Idle));

        addTransition(IntakeLeft, Idle, new InstantCommand(() -> {
            i.requestTransition(Intake.IntakeState.Idle);
            co.setShouldEndIntakeSequence(true); //We shouldn't directly request Idle because the intaking subroutine occurs with the shouldEndIntakeSequence flag

            if(t.isInState(Turret.TurretState.ActiveTracking) && !t.isTransitioning()) {
                t.requestTransition(Turret.TurretState.ActiveTracking);
            }
        }));

        //If we're already intaking right, it's safe to go back to idle
        addSafeTransitionCondition(IntakeRight, Idle, () -> i.isInState(Intake.IntakeState.LeftSideRunning));

        addCommutativeTransition(IntakeRight, IntakeLeft,
                new InstantCommand(() -> {
                    i.requestTransition(Intake.IntakeState.LeftSideRunning);
                    co.requestTransition(Conveyor.ConveyorState.StartIntakeLeft);
                }),
                new InstantCommand(() -> {
                    i.requestTransition(Intake.IntakeState.RightSideRunning);
                    co.requestTransition(Conveyor.ConveyorState.StartIntakeRight);
                })
        );
        
        //If we're already intaking, it's safe to go to the other intaking state
        addSafeTransitionCondition(IntakeRight, IntakeLeft, () -> i.isInState(Intake.IntakeState.RightSideRunning));
        addSafeTransitionCondition(IntakeLeft, IntakeRight, () -> i.isInState(Intake.IntakeState.LeftSideRunning));

        AtomicBoolean wasEjecting = new AtomicBoolean(false);
        setContinuousCommand(IntakeRight, new ParallelCommandGroup(
                new RunCommand(() -> {
                    if(co.isInState(Conveyor.ConveyorState.EjectFromLeft, Conveyor.ConveyorState.EjectFromRight)) {
                            t.requestTransition(Turret.TurretState.Ejecting);
                            wasEjecting.set(true);
                    } else if(wasEjecting.get()) {
                        //we're done ejecting
                        t.requestTransition(Turret.TurretState.ActiveTracking);
                        wasEjecting.set(false);}
                }),
                getLightControlCommand() //Run light control while intaking to indicate number of balls
        ));

        setContinuousCommand(IntakeLeft, new ParallelCommandGroup(
                new RunCommand(() -> {
                    if(co.isInState(Conveyor.ConveyorState.EjectFromLeft, Conveyor.ConveyorState.EjectFromRight)) {
                        t.requestTransition(Turret.TurretState.Ejecting);
                        wasEjecting.set(true);
                    } else if(wasEjecting.get()) {
                        //we're done ejecting
                        t.requestTransition(Turret.TurretState.ActiveTracking);
                        wasEjecting.set(false);}
                }),
                getLightControlCommand() //Run light control while intaking to indicate number of balls
        ));

        addTransition(Idle, AttemptShooting, new InstantCommand());
        setContinuousCommand(AttemptShooting, new InstantCommand(() -> {
            if(t.getCurrentState() == Turret.TurretState.LockedIn) {
                requestTransition(Shoot);
            } else requestTransition(Idle);
        }));

        //Only start shooting if the conveyor and intakes are idle
        addSafeTransitionCondition(Idle, AttemptShooting, () -> true);

        addTransition(AttemptShooting, Shoot, new InstantCommand(() -> {
            co.requestTransition(Conveyor.ConveyorState.StartShooting);
        }));
        
        //If we're already attempting to shoot, it is safe to start shooting
        addSafeTransitionCondition(AttemptShooting, Shoot, () -> true);

        //Climb lock-out logic
        //No transition conditions here because if the bot is in idle everything else has already been requested to stop
        addTransition(Idle, Climb, new InstantCommand(() ->
        {
            c.requestTransition(Climber.ClimberState.Phase1);
            t.requestTransition(Turret.TurretState.ClimbLock);
            l.requestTransition(Lights.LEDState.Climbing);
        }));

    }

    /**
     * Different method that only allows safe transitions to be made so that the robot doesn't get out of sync with itself
     */
    @Override
    public boolean requestTransition(RobotState state) {
        SimpleTransition<RobotState> t = new SimpleTransition<RobotState>(getParentState(), state); //Generate a SimpleTransition object to describe the current transition
        List<BooleanSupplier> conditions = new ArrayList<>(); //Find the matching transition in the list of conditions

        for(Map.Entry<SimpleTransition<RobotState>, List<BooleanSupplier>> c : safeConditions.entrySet()) {
            SimpleTransition<RobotState> condition = c.getKey();

            if(condition.startState == t.startState && condition.endState == t.endState && condition.interruptionState == t.interruptionState) {
                    conditions = c.getValue();
                    break;
            }
        }

        try {
            for(BooleanSupplier s : conditions) {
                if(!s.getAsBoolean()) return false; //Return false if any of the conditions are not meant
            }
        } catch (Exception E) {
            //Catch statement for if no conditions matching the transtition were found.
            //An error will be printed to the console (along with a stacktrace), but the transition will still run

            System.out.println("FAILED TO FIND CONDITIONS FOR THE REQUESTED TRANSITION (in RobotManager.java)");
            System.out.println("DETAILS: START STATE: " + getParentState()  + ", END STATE: " + state);
            E.printStackTrace();
        }
    
        return super.requestTransition(state);
    }

    /**
     * Create a new set of conditions for a certain transition
     * @param startState the state from which the transition begins
     * @param endState the goal state of the transition
     * @param conditions the conditions that must be met for the Robot Manager to allow the transition to occur
     */
    private void addSafeTransitionCondition(RobotState startState, RobotState endState, BooleanSupplier... conditions) {
        safeConditions.put(new SimpleTransition<RobotState>(startState, endState), List.of(conditions));
    }

    @Override
    public void update() {
        //We'll run an instantaneous transition back to
        Constants.RobotEnabled botEnabledStatus = Constants.botEnabledStatus;

        if(botEnabledStatus == Constants.RobotEnabled.Disabled && botEnabledStatus != prevBotEnabledStatus) {
            runInstantaneousTransition(Idle, () -> {}); // All the subsystems will transition themselves to idle
        }

        prevBotEnabledStatus = botEnabledStatus;
    }

    /**
     * Get a new command that will control the lights to display the ball count
     * @return the command to run
     */
    private Command getLightControlCommand() {
        return new RunCommand(() -> {
            int ballCount = co.getNumBalls();

            if(isInState(Idle, IntakeRight, IntakeLeft, AttemptShooting)) {
                if(!prevShouldDisplayBallCount)  {
                    l.requestTransition(Lights.LEDState.Default);
                }

                if(ballCount == 1 && ballCount != prevBallCount) {
                    l.requestTransition(Lights.LEDState.OneBall);
                } else if(ballCount == 2 && ballCount != prevBallCount) {
                    l.requestTransition(Lights.LEDState.TwoBall);
                }

                prevShouldDisplayBallCount = true;
            } else prevShouldDisplayBallCount = false;

            prevBallCount = ballCount;
        });
    }

    public void advanceClimbState() {
        if(getCurrentState() == Climb) {
            switch(c.getCurrentState()) {
                case Phase1:
                    c.requestTransition(Climber.ClimberState.Phase2);
                    return;
                case Phase2:
                    c.requestTransition(Climber.ClimberState.Phase3);
                    return;
            }
        } else requestTransition(Climb);
    }

    @Override
    public String getName() {
        return "Robot Manager";
    }


    public enum RobotState {
        Undetermined, Idle, IntakeLeft, IntakeRight, EjectBottom, AttemptShooting, Shoot, Climb, Test
    }

    @Override
    public void additionalSendableData(SendableBuilder builder) {

    }
}
