package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ShamLib.SMF.StatedSubsystem;
import frc.robot.ShamLib.motors.MotionMagicTalonFX;

import java.util.Map;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Climber.*;
import static frc.robot.subsystems.Climber.*;
import static frc.robot.subsystems.Climber.ClimberState.*;

public class Climber extends StatedSubsystem<ClimberState> {

    private final MotionMagicTalonFX motor1 = new MotionMagicTalonFX(MOTOR_1_ID, climberGains, climberRatio, MAX_VEL, MAX_ACCEL);
    private final WPI_TalonFX motor2 = new WPI_TalonFX(MOTOR_2_ID);

    private final Solenoid brake = new Solenoid(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, SOLENOID_1_PORT);

    public Climber() {
        super(ClimberState.class);

        //Set up follower motors
        motor2.follow(motor1);
        motor2.setInverted(InvertType.FollowMaster);

        enableMotorBraking();

        addDetermination(Undetermined, Idle, new InstantCommand(() -> {
            brake();
            resetClimberPos();
            setClimberTarget(LOWERED_POS);
        }));

        addTransition(Idle, Up, new InstantCommand(() -> {
            unBrake();
        }).andThen(new WaitCommand(0.5)).andThen(new InstantCommand(() -> setClimberTarget(RIASED_POS))));

        addTransition(Up, Idle, new InstantCommand(() -> {
            unBrake();
        }).andThen(new WaitCommand(0.5)).andThen(new InstantCommand(() -> setClimberTarget(LOWERED_POS)).andThen(new WaitCommand(1)).andThen(new InstantCommand(() -> {
            setManualPower(0);
            brake();
        }))));

        addTransition(Idle, Test, new InstantCommand(() -> {
            disableMotorBraking();
            setManualPower(0);
            unBrake();
        }));
        addTransition(Test, Idle, new InstantCommand(() -> {
            enableMotorBraking();
            brake();
        }));
    }

    @Override
    public void update() {
        if(!climberBusy() && !isBraked() && !isInState(Test) && !isTransitioning()) {
            brake();
            motor1.set(ControlMode.PercentOutput, 0);

        }
        // } else if(climberBusy() && isBraked()  && !isInState(Test) && !isTransitioning()) {
        //     unBrake();
        // }
    }

    @Override
    protected void onEnable() {  
    }

    public void setManualPower(double power) {
        motor1.setManualPower(power);
    }

    @Override
    protected void onDisable() {
        if(getCurrentState() == Test) {
            runInstantaneousTransition(Idle, () -> {
                enableMotorBraking();
            });
        }
    }

    public void climbTest() {
        runInstantaneousTransition(Test, () -> {
            disableMotorBraking();
            unBrake();
            setManualPower(0);
        });
    }

    private void enableMotorBraking() {motor1.setNeutralMode(NeutralMode.Brake); motor2.setNeutralMode(NeutralMode.Brake);}
    private void disableMotorBraking() {motor1.setNeutralMode(NeutralMode.Coast); motor2.setNeutralMode(NeutralMode.Coast);}

    private void setClimberTarget(double targetInches) {motor1.setTarget(targetInches);}

    private double getClimberTarget() {return motor1.getTarget();}

    private double getClimberPos() {
        return motor1.getPosition();
    }

    public boolean climberBusy() {
        return Math.abs(getClimberPos() - getClimberTarget()) > TOLERANCE;
    }

    private double getClimberVelo() {
        return motor1.getVelocity();
    }

    public void resetClimberPos() {motor1.setSelectedSensorPosition(0);}

    private void setManualPower(Direction direction) {
        double power = MINIMUM_POWER;
        power *= direction == Direction.Up ? 1 : -1;

        if(direction == Direction.Stopped) {
            motor1.set(0);
            return;
        }

        motor1.set(power);
    }

    private void brake() {brake.set(false);}
    private void unBrake() {brake.set(true);}
    private boolean isBraked() {return !brake.get();}

    @Override
    public String getName() {return "climber";}

    @Override
    protected void additionalSendableData(SendableBuilder builder) {
        builder.addBooleanProperty("brake on", () -> isBraked(), null);
        builder.addDoubleProperty("current velo (in per sec)", () -> getClimberVelo(), null);
        builder.addDoubleProperty("current velo (tick per sec)", () -> motor1.getSelectedSensorVelocity(), null);
        builder.addDoubleProperty("goal pos", () -> getClimberTarget(), null);
        builder.addDoubleProperty("current pos", () -> getClimberPos(), null);
    }

    @Override
    public Map<String, Sendable> additionalSendables() {
        return Map.of(
                // "test/reset climber pos", new InstantCommand(() -> {if(getCurrentState() == Test) resetClimberPos();}),
                "test/unbrake", new InstantCommand(() -> unBrake()),
                "test/brake", new InstantCommand(() -> brake())
                // "test/raise climber", new InstantCommand(() -> {if(getCurrentState() == Test) setManualPower(Direction.Up);}),
                // "test/lower climber", new InstantCommand(() -> {if(getCurrentState() == Test) setManualPower(Direction.Down);}),
                // "test/stop climber", new InstantCommand(() -> {if(getCurrentState() == Test) setManualPower(Direction.Stopped);})
        );
    }

    public enum ClimberState {
        Undetermined, Idle, Up, Test
    }

    private enum Direction {
        Up, Down, Stopped
    }
}
