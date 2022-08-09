package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.turret.ActiveTracking;
import frc.robot.commands.turret.DetermineTurretState;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;
import frc.robot.util.hardware.Limelight;
import frc.robot.util.hardware.MagneticLimitSwitch;
import frc.robot.util.math.InterpLUT;

import java.util.concurrent.atomic.AtomicBoolean;

import static com.ctre.phoenix.motorcontrol.InvertType.*;
import static frc.robot.Constants.Turret.*;
import static frc.robot.subsystems.Turret.RotarySpeed.*;
import static frc.robot.subsystems.Turret.TurretState.*;
import static java.lang.Math.abs;

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

    private final SimpleMotorFeedforward rotaryFeedForward = new SimpleMotorFeedforward(ROTARY_KS, ROTARY_KV);
    private final ProfiledPIDController rotaryPID = new ProfiledPIDController(ROTARY_KP, ROTARY_KI, ROTARY_KD, NORMAL_ROTARY_CONSTRAINTS);
    private boolean rotaryControlLoopEnabled = true;
    private double prevRotaryAngle;
    private RotarySpeed rotarySpeed = Normal;

    private final SimpleMotorFeedforward hoodFeedForward = new SimpleMotorFeedforward(HOOD_KS, HOOD_KV);
    private final ProfiledPIDController hoodPID = new ProfiledPIDController(HOOD_KP, HOOD_KI, HOOD_KD,
            new TrapezoidProfile.Constraints(HOOD_MAX_VEL, HOOD_MAX_ACCEL));
    private boolean hoodControlLoopEnabled = true;

    private boolean hoodOverextended = false;
    private boolean rotaryOverextended = false;

    private boolean trustResetting = false; //Whether or not the bot should just assume everything is reset properly

    public Turret() {
        super(TurretState.class);

        Constants.configureMotor(rotaryMotor, true);
        Constants.configureMotor(hoodMotor, true);
        Constants.configureMotor(flywheel1Motor, false, false, true);
        Constants.configureMotor(flywheel2Motor, false, false, true);

        flywheel2Motor.follow(flywheel1Motor);
        flywheel2Motor.setInverted(OpposeMaster);

        //Set the tolerance for the PID loop to say it is not busy
        rotaryPID.setTolerance(ROTARY_TOLERANCE);
        flywheelPID.setTolerance(FLYWHEEL_TOLERANCE);
        hoodPID.setTolerance(HOOD_TOLERANCE);

        //Setup lookup tables for velocity and angle control
        RPMLUT.add(0, 0);
        RPMLUT.add(1, 1);
        RPMLUT.createLUT();

        HoodAngleLUT.add(0, 0);
        HoodAngleLUT.add(1, 1);
        HoodAngleLUT.createLUT();

        getRotaryAngle = () -> Rotation2d.fromDegrees(getRotaryAngle());
        getLimelightXOffsetAngle = () -> Rotation2d.fromDegrees(getLimelightXOffsetDegrees());

        addDetermination(Undetermined, Idle, new InstantCommand(() -> {
            resetHoodPos(0);
            resetRotaryPos(0);
            setRotarySpeed(Normal);
            resetHoodPID();
            resetRotaryPID();
            resetFlywheelPID();
            turnOffLimelight();
            setFlywheelTargetRPM(0);
        }));

        setContinuousCommand(Idle, new InstantCommand(() -> {
            if(!trustResetting) {
                requestTransition(Resetting);
            }
        }));

        addCommutativeTransition(Idle, ActiveTracking, new InstantCommand((this::turnOnLimelight)), new InstantCommand((this::turnOffLimelight)));


        //TODO: GET ODO POSE

        AtomicBoolean lockedIn = new AtomicBoolean(false);

        setContinuousCommand(ActiveTracking, new ActiveTracking(this, RPMLUT, HoodAngleLUT, (value) -> lockedIn.set(value)));

        addFlagState(ActiveTracking, LockedIn, () -> lockedIn.get());

        addTransition(Idle, Resetting, new InstantCommand());
        setContinuousCommand(Resetting, new DetermineTurretState(this));
        addTransition(Resetting, Idle, new InstantCommand());

        addTransition(Idle, ClimbLock, new InstantCommand(() -> setRotaryTargetAngle(0)));
    }

    public void setFlywheelTargetRPM(double rpm) {
        flywheelPID.setSetpoint(rpm);
    }

    public void setRotaryTargetAngle(double degrees) {
        double clippedDegrees = ROTARY_RANGE.clipToRange(degrees);
        if(clippedDegrees != degrees) rotaryOverextended = true;
        else if (rotaryOverextended) rotaryOverextended = false;
        rotaryPID.setGoal(clippedDegrees);
    }

    public void setHoodTargetAngle(double degrees) {
        double clippedDegrees = HOOD_RANGE.clipToRange(degrees);
        if(clippedDegrees != degrees) hoodOverextended = true;
        else if (hoodOverextended) hoodOverextended = false;
        hoodPID.setGoal(degrees);
    }

    @Override
    public void onEnable() {
        resetAllPIDs();
    }

    @Override
    public void onDisable() {
        runInstantaneousTransition(
                Idle,
                () -> {
                    setHoodTargetAngle(getHoodAngle());
                    setFlywheelTargetRPM(0);
                    setHoodTargetAngle(getHoodAngle());
                    turnOffLimelight();
                }
        );
    }

    public void resetAllPIDs() {
        resetFlywheelPID();
        resetHoodPID();
        resetHoodPID();
    }

    public void resetFlywheelPID() {
        flywheelPID.reset();
    }

    public void resetRotaryPID() {
        rotaryPID.reset(getRotaryAngle(), getRotaryVelo());
    }

    public void resetHoodPID() {
        hoodPID.reset(getHoodAngle(), getHoodVelo());
    }

    @Override
    public void update() {

        //Flywheel calculations
        updateFlywheel();

        //Rotary calculations
        updateRotary();

        //Hood calculations
        updateHood();
    }

    private void updateFlywheel() {
        double flywheelFFOutput = flywheelFeedForward.calculate(flywheelPID.getSetpoint());
        double flywheelPIDOutput = flywheelPID.calculate(getFlywheelRPM());

        flywheel1Motor.setVoltage(flywheelFFOutput + flywheelPIDOutput);
    }

    private void updateRotary() {
        if(rotaryControlLoopEnabled) {
            double rotaryFFOutput = rotaryFeedForward.calculate(rotaryPID.getSetpoint().velocity);
            double rotaryPIDOutput = rotaryPID.calculate(getRotaryAngle());

            double rotarySum = rotaryFFOutput + rotaryPIDOutput;
            if(limSwitch1.isTripped() && rotarySum > 0) rotarySum = 0;
            else if(limSwitch2.isTripped() && rotarySum < 0) rotarySum = 0;

            rotaryMotor.setVoltage(rotarySum);
        }

        prevRotaryAngle = getRotaryAngle();
    }

    private void updateHood() {
        if(hoodControlLoopEnabled) {
            double hoodFFOutput = hoodFeedForward.calculate(hoodPID.getSetpoint().velocity);
            double hoodPIDOutput = hoodPID.calculate(getHoodAngle());

            hoodMotor.setVoltage(hoodFFOutput + hoodPIDOutput);
        }
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
    public double getRotaryAngle() {
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

    public boolean isSensor1Pressed() {return limSwitch1.isTripped();}
    public boolean isSensor2Pressed() {return limSwitch2.isTripped();}
    public double getPrevRotaryAngle() {return prevRotaryAngle;}

    public boolean isRotaryBusy() {return rotaryPID.atGoal();}
    public boolean isFlywheelBusy() {return flywheelPID.atSetpoint();}
    public boolean isHoodBusy() {return hoodPID.atGoal();}

    public boolean isRotaryOverextended() {return rotaryOverextended;}
    public boolean isHoodOverextended() {return hoodOverextended;}

    public void setRotarySpeed(RotarySpeed speed) {
        rotarySpeed = speed;
        if(speed==Normal) {
            rotaryPID.setConstraints(NORMAL_ROTARY_CONSTRAINTS);
        } else if(speed==Search) {
            rotaryPID.setConstraints(SEARCH_ROTARY_CONSTRAINTS);
        }
    }

    public RotarySpeed getRotarySpeed() {return rotarySpeed;}

    //Reset methods
    public void resetHoodPos(double degrees) {
        hoodMotor.setSelectedSensorPosition(degrees * (485.0 / 18.0) * 2048);
    }

    public void resetRotaryPos(double degrees) {
        rotaryMotor.setSelectedSensorPosition(degrees * (140.0 / 40.0) / 1.0 * 2048);
    }

    //Disabling control loops
    public void disableHoodControlLoops() {hoodControlLoopEnabled = false;}
    public void enableHoodControlLoops() {hoodControlLoopEnabled = true;}
    public void disableRotaryControlLoops() {hoodControlLoopEnabled = false;}
    public void enableRotaryControlLoops() {hoodControlLoopEnabled = true;}

    //Diagnostic access methods
    public double getFlywheelTarget() {return flywheelPID.getSetpoint();}
    public double getFlywheelError() {return abs(getFlywheelTarget()-getFlywheelRPM());}
    public double getHoodTarget() {return hoodPID.getSetpoint().position;}
    public double getHoodError() {return abs(getHoodTarget() - getHoodAngle());}
    public double getRotaryTarget() {return rotaryPID.getSetpoint().position;}
    public double getRotaryError() {return abs(getRotaryTarget() - getRotaryAngle());}

    //Limelight access methods
    public void turnOnLimelight() {limelight.setOn();}
    public void turnOffLimelight() {limelight.setOff();}
    public boolean doesLimelightHaveTarget() {return limelight.hasTarget();}
    public double getLimelightXOffsetDegrees() {return limelight.getXOffset().getDegrees();}
    public double getLimelightYOffsetDegrees() {return limelight.getYOffset().getDegrees();}
    public double getLimelightDistanceFromCenter() {return limelight.getDistanceFromCenter();}

    @Override
    public String getName() {return "turret";}

    @Override
    public void additionalSendableData(SendableBuilder builder) {
        builder.addDoubleProperty("flywheel velo", this::getFlywheelRPM, null);
        builder.addDoubleProperty("flywheel target", this::getFlywheelTarget, null);
        builder.addDoubleProperty("flywheel error", this::getFlywheelError, null);
        builder.addDoubleProperty("rotary degrees", this::getRotaryAngle, null);
        builder.addDoubleProperty("rotary velo", this::getRotaryVelo, null);
        builder.addDoubleProperty("rotary target", this::getRotaryTarget, null);
        builder.addDoubleProperty("rotary error", this::getRotaryError, null);
        builder.addDoubleProperty("hood degrees", this::getHoodAngle, null);
        builder.addDoubleProperty("hood velo", this::getHoodVelo, null);
        builder.addDoubleProperty("hood target", this::getHoodTarget, null);
        builder.addDoubleProperty("hood error", this::getHoodError, null);
        builder.addBooleanProperty("sensor 1 pressed", this::isSensor1Pressed, null);
        builder.addBooleanProperty("sensor 2 pressed", this::isSensor2Pressed, null);

        builder.addDoubleProperty("limelight distance from target", this::getLimelightDistanceFromCenter, null);
    }

    public enum TurretState {
        Undetermined, Idle, Resetting, Ejecting, ActiveTracking, LockedIn, ClimbLock
    }

    public enum RotarySpeed {
        Normal, Search
    }

    //TODO: REMOVE DEBUG FUNCTIONS
    public void setFlywheelPower(double power) {
        flywheel1Motor.set(power);
    }

    public void setHoodPower(double power) {
        hoodMotor.set(power);
    }

    public void setRotaryPower(double power) {
        rotaryMotor.set(power);
    }


}
