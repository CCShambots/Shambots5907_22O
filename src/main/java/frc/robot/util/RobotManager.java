package frc.robot.util;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;

import static frc.robot.util.RobotManager.RobotState.*;

public class RobotManager extends StatedSubsystem<RobotManager.RobotState> {
    private Drivetrain dt;
    private Intake i;
    private Conveyor co;
    private Turret t;
    private Climber c;
    private Lights l;

    public RobotManager(Drivetrain dt, Intake i, Conveyor co, Turret t, Climber c, Lights l) {
        super(RobotState.class);

        this.dt = dt;
        this.i = i;
        this.co = co;
        this.t = t;
        this.c = c;
        this.l = l;

        //Determination Logic
        addDetermination(Undetermined, Idle, new ParallelCommandGroup(
                dt.waitForState(dt.getEntryState()),
                i.waitForState(i.getEntryState()),
                co.waitForState(co.getEntryState()),
                t.waitForState(t.getEntryState()),
                c.waitForState(c.getEntryState()),
                l.waitForState(l.getEntryState())
        ));

        //Test logic
        addTransition(Idle, Test, new InstantCommand(() -> {
            c.requestTransition(Climber.ClimberState.Test);
            l.requestTransition(Lights.LEDState.Testing);
        }));

        //Manual bottom eject logic
        addTransition(Idle, EjectBottom, new InstantCommand(() -> {
            i.requestTransition(Intake.IntakeState.Ejecting);
            co.requestTransition(Conveyor.ConveyorState.EjectAll);
        }));

        setContinuousCommand(EjectBottom, new WaitCommand(Constants.Intake.MANUAL_EJECT_TIME).andThen(new InstantCommand(() -> requestTransition(Idle))));

        addTransition(EjectBottom, Idle, new InstantCommand(() -> {
            i.requestTransition(Intake.IntakeState.Idle);
            co.requestTransition(Conveyor.ConveyorState.Idle);
        }));

        //Intake Cycle logic
        addTransition(Idle, IntakeRight, new InstantCommand(() -> {
            i.requestTransition(Intake.IntakeState.RightSideRunning);
            co.requestTransition(Conveyor.ConveyorState.StartIntakeRight);
        }));

        addTransition(IntakeRight, Idle, new InstantCommand(() -> {
            
        }));


        //Climb lock-out logic
        addTransition(Idle, Climb, new InstantCommand(() ->
        {
            c.requestTransition(Climber.ClimberState.Phase1);
            t.requestTransition(Turret.TurretState.ClimbLock);
        }));
    }

    @Override
    public void update() {

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
        Undetermined, Idle, IntakeLeft, IntakeRight, EjectBottom, EjectTop, Shoot, Climb, Test
    }

    @Override
    public void additionalSendableData(SendableBuilder builder) {

    }
}
