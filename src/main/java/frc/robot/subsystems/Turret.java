package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;
import frc.robot.util.hardware.Limelight;
import frc.robot.util.hardware.MagneticLimitSwitch;
import frc.robot.util.math.InterpLUT;

import static com.ctre.phoenix.motorcontrol.InvertType.*;
import static frc.robot.Constants.Turret.*;
import static frc.robot.subsystems.Turret.TurretState.*;

public class Turret extends StatedSubsystem<Turret.TurretState> {
    private final WPI_TalonFX rotaryMotor = new WPI_TalonFX(ROTARY_MOTOR_ID);
    private final WPI_TalonFX hoodMotor = new WPI_TalonFX(HOOD_MOTOR_ID);
    private final WPI_TalonFX flywheel1Motor = new WPI_TalonFX(FLYWHEEL_MOTOR1_ID);
    private final WPI_TalonFX flywheel2Motor = new WPI_TalonFX(FLYWHEEL_MOTOR2_ID);

    private final MagneticLimitSwitch limSwitch1 = new MagneticLimitSwitch(LIM_SWITCH_1_ID);
    private final MagneticLimitSwitch limSwitch2 = new MagneticLimitSwitch(LIM_SWITCH_2_ID);

    private final Limelight limelight = Limelight.getInstance();

    private final InterpLUT RPMLUT = new InterpLUT();
    private final InterpLUT HoodAngleLUT = new InterpLUT();

    private final SimpleMotorFeedforward flywheelFeedForward = new SimpleMotorFeedforward(FLYWHEEL_KS, FLYWHEEL_KV);
    private final PIDController flywheelPID = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
    private double flywheelTargetVelo = 0;

    private final SimpleMotorFeedforward rotaryFeedForward = new SimpleMotorFeedforward(ROTARY_KS, ROTARY_KV);
    private final ProfiledPIDController rotaryPID = new ProfiledPIDController(ROTARY_KP, ROTARY_KI, ROTARY_KD,
            new TrapezoidProfile.Constraints(ROTARY_MAX_VEL, ROTARY_MAX_ACCEL));

    private final SimpleMotorFeedforward hoodFeedForward = new SimpleMotorFeedforward(HOOD_KS, HOOD_KV);
    private final ProfiledPIDController hoodPID = new ProfiledPIDController(HOOD_KP, HOOD_KI, HOOD_KD,
            new TrapezoidProfile.Constraints(HOOD_MAX_VEL, HOOD_MAX_ACCEL));

    public Turret() {
        super(TurretState.class);

        //TODO: Make config factory default true later
        Constants.configureMotor(rotaryMotor, false);
        Constants.configureMotor(hoodMotor, false);
        Constants.configureMotor(flywheel1Motor, false);
        Constants.configureMotor(flywheel2Motor, false);

        flywheel2Motor.follow(flywheel1Motor);
        flywheel2Motor.setInverted(OpposeMaster);


        RPMLUT.add(0, 0);
        RPMLUT.createLUT();

        HoodAngleLUT.add(0, 0);
        HoodAngleLUT.createLUT();

        //TODO: Make actual state finding
        addDetermination(Undetermined, Idle, new InstantCommand());
    }

    public void setFlywheelTargetRPM(double rpm) {
        flywheelPID.setSetpoint(rpm);
    }

    public void setRotaryTargetAngle(double degrees) {

        rotaryPID.setGoal(degrees);
    }

    public void setHoodTargetAngle(double degrees) {
        hoodPID.setGoal(degrees);
    }

    @Override
    public void update() {

        //Flywheel calculations
        double flywheelFFOutput = flywheelFeedForward.calculate(flywheelPID.getSetpoint());
        double flywheelPIDOutput = flywheelPID.calculate(getFlywheelRPM());

        flywheel1Motor.setVoltage(flywheelFFOutput + flywheelPIDOutput);

        //Rotary calculations
        double rotaryFFOutput = rotaryFeedForward.calculate(rotaryPID.getSetpoint().velocity);
        double rotaryPIDOutput = rotaryPID.calculate(getRotaryPos());

        rotaryMotor.setVoltage(rotaryFFOutput + rotaryPIDOutput);

        //Hood calculations
        double hoodFFOutput = hoodFeedForward.calculate(hoodPID.getSetpoint().velocity);
        double hoodPIDOutput = hoodPID.calculate(getHoodAngle());

        rotaryMotor.setVoltage(hoodFFOutput + hoodPIDOutput);


    }

    /**
     * @return RPM
     */
    public double getFlywheelRPM() {
        return
            flywheel1Motor.getSelectedSensorVelocity() * // Ticks/100ms
            10 * // Ticks/sec
            2048 // Rotations/sec
        ;
    }

    /**
     * @return deg
     */
    public double getRotaryPos() {
        return
            rotaryMotor.getSelectedSensorPosition() //Ticks
            / 2048 * //Rotations
            1 * //Gear ratio on versaplanetary
            (40.0 / 140) //Rotations
        ;
    }

    /**
     * @return Deg/sec
     */
    public double getRotaryVelo() {
        return
            rotaryMotor.getSelectedSensorPosition() * //Ticks/100ms
                    10  //Ticks
                    /2048 * //Rotations
                    1 * //Gear ratio on versaplanetary
                    (40.0 / 140) //Degrees
        ;
    }

    /**
     * @return deg
     */
    public double getHoodAngle() {
        return
                hoodMotor.getSelectedSensorPosition() //Ticks
                /2048 * //Rotations
                (18.0 / 485.0) //Degrees
        ;
    }

    /**
     * @return deg/sec
     */
    public double getHoodVelo() {
        return
                hoodMotor.getSelectedSensorPosition() //Ticks/100ms
                        * 10 //Ticks/sec
                        /2048 * //Rotations/sec
                        (18.0 / 485.0) //Degrees/sec
                ;
    }

    @Override
    public String getName() {
        return "turret";
    }

    @Override
    public void additionalSendableData(SendableBuilder builder) {
        builder.addDoubleProperty("flywheel velo", this::getFlywheelRPM, null);
        builder.addDoubleProperty("rotary degrees", this::getRotaryPos, null);
        builder.addDoubleProperty("rotary velo", this::getRotaryVelo, null);
        builder.addDoubleProperty("hood degrees", this::getHoodAngle, null);
        builder.addDoubleProperty("hood velo", this::getHoodVelo, null);
    }

    public enum TurretState {
        Undetermined, Idle
    }

}
