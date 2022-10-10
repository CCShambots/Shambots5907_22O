package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;
import frc.robot.util.hardware.MotorConfiguration;

import java.util.Map;

import static edu.wpi.first.wpilibj.DoubleSolenoid.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.Climber.*;
import static frc.robot.subsystems.Climber.*;
import static frc.robot.subsystems.Climber.ClimberState.*;

public class Climber extends StatedSubsystem<ClimberState> {

    private final WPI_TalonFX motor1 = new WPI_TalonFX(MOTOR_1_ID);
    private final WPI_TalonFX motor2 = new WPI_TalonFX(MOTOR_2_ID);

    private boolean motorControlEnabled = true;

    private final DoubleSolenoid solenoid = new DoubleSolenoid(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, SOLENOID_1_PORT, SOLENOID_2_PORT);

    private final SimpleMotorFeedforward ffController = new SimpleMotorFeedforward(KS, KV);
    private final ProfiledPIDController pidController = new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL));

    public Climber() {
        super(ClimberState.class);

        MotorConfiguration.configureBasicMotor(motor1);
        MotorConfiguration.configureBasicMotor(motor2);

        motor2.follow(motor1);
        motor2.setInverted(InvertType.OpposeMaster);

        Constants.optimizeFollowerMotor(motor2);

        addDetermination(Undetermined, Idle, new InstantCommand(() -> {
            extendSolenoid();
            resetClimberPos();
            setClimberTarget(LOWERED_POS);
        }));

        addTransition(Idle, Phase1, new InstantCommand(() -> setClimberTarget(RIASED_POS)));

        addTransition(Phase1, Phase2, new InstantCommand(() -> {
            setClimberTarget(LOWERED_POS);
            retractSolenoid();
        }));

        addTransition(Phase2, Phase3, new InstantCommand(() -> setClimberTarget(RIASED_POS)));

        addTransition(Idle, Test, new InstantCommand(this::disableMotorControl));
        addTransition(Test, Idle, new InstantCommand(this::enableMotorControl));
    }

    @Override
    public void update() {
        if(motorControlEnabled) {
            double ffOutput = ffController.calculate(pidController.getSetpoint().velocity);
            double pidOutput = pidController.calculate(getClimberPos());

            motor1.setVoltage(ffOutput + pidOutput);
        }
    }

    @Override
    protected void onEnable() {
        resetPID();
    }

    @Override
    protected void onDisable() {
        if(getCurrentState() == Test) {
            runInstantaneousTransition(Idle, () -> {
                enableMotorControl();
            });
        }
    }

    private void enableMotorControl() {motorControlEnabled = true;}
    private void disableMotorControl() {motorControlEnabled = false;}
    private boolean isMotorControlEnabled() {return motorControlEnabled;}

    private void resetPID() {pidController.reset(getClimberPos());}

    private void setClimberTarget(double targetInches) { pidController.setGoal(targetInches);}
    private double getClimberTarget() {return pidController.getSetpoint().position;}

    private double getClimberPos() {
        return motor1.getSelectedSensorPosition() * 2048 * //Motor revolutions
        12.0/50.0 * // Stage 1
        14.0/60.0 * //Stage 2 : revolutions of sprocket
        1.751 * Math.PI; //Pitch diameter (1.751") :  distance traveled by chain
    }

    private double getClimberVelo() {
        return motor1.getSelectedSensorPosition() * 10 * 2048 * //Motor revolutions
                12.0/50.0 * // Stage 1
                14.0/60.0 * //Stage 2 : revolutions of sprocket
                1.751 * Math.PI; //Pitch diameter (1.751") :  distance traveled by chain
    }

    private void resetClimberPos() {motor1.setSelectedSensorPosition(0);}

    private void setManualPower(Direction direction) {
        double power = MINIMUM_POWER;
        power *= direction == Direction.Up ? 1 : -1;

        if(direction == Direction.Stopped) {
            motor1.set(0);
            return;
        }

        motor1.set(power);
    }

    private void extendSolenoid() {solenoid.set(kForward);}
    private void retractSolenoid() {solenoid.set(kReverse);}
    private Value getSolenoidState() {return solenoid.get();}

    @Override
    public String getName() {return "climber";}

    @Override
    protected void additionalSendableData(SendableBuilder builder) {
        // builder.addStringProperty("solenoid state", () -> getSolenoidState().name(), null);
        // builder.addBooleanProperty("motor control enabled", () -> isMotorControlEnabled(), null);
        // builder.addDoubleProperty("current velo (in/sec)", () -> getClimberVelo(), null);
        // builder.addDoubleProperty("goal velo (in/sec)", () -> pidController.getSetpoint().velocity, null);
        // builder.addDoubleProperty("goal pos", () -> getClimberTarget(), null);
        // builder.addDoubleProperty("current pos", () -> getClimberPos(), null);
    }

    @Override
    public Map<String, Sendable> additionalSendables() {
        return Map.of(
                // "test/reset climber pos", new InstantCommand(() -> {if(getCurrentState() == Test) resetClimberPos();}),
                // "test/extend piston", new InstantCommand(() -> {if(getCurrentState() == Test) extendSolenoid();}),
                // "test/retract piston", new InstantCommand(() -> {if(getCurrentState() == Test) retractSolenoid();}),
                // "test/raise climber", new InstantCommand(() -> {if(getCurrentState() == Test) setManualPower(Direction.Up);}),
                // "test/lower climber", new InstantCommand(() -> {if(getCurrentState() == Test) setManualPower(Direction.Down);}),
                // "test/stop climber", new InstantCommand(() -> {if(getCurrentState() == Test) setManualPower(Direction.Stopped);})
        );
    }

    public enum ClimberState {
        Undetermined, Idle, Phase1, Phase2, Phase3, Test
    }

    private enum Direction {
        Up, Down, Stopped
    }
}
