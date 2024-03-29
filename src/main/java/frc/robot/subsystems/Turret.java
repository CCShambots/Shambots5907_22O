package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StatedSubsystem;
import frc.robot.commands.turret.ActiveTracking;
import frc.robot.commands.turret.AutoEjecting;
import frc.robot.commands.turret.DetermineTurretState;
import frc.robot.util.hardware.Limelight;
import frc.robot.util.hardware.MagneticLimitSwitch;
import frc.robot.util.hardware.MotorConfiguration;
import frc.robot.util.math.InterpLUT;

import java.util.concurrent.atomic.AtomicBoolean;

import static com.ctre.phoenix.motorcontrol.InvertType.*;
import static frc.robot.Constants.Turret.*;
import static frc.robot.subsystems.Turret.RotarySpeed.*;
import static frc.robot.subsystems.Turret.TurretState.*;
import static java.lang.Math.abs;

public class Turret extends StatedSubsystem<Turret.TurretState> {
    private final WPI_TalonFX rotaryMotor = new WPI_TalonFX(ROTARY_MOTOR_ID, "Drivetrian");
    private final WPI_TalonFX hoodMotor = new WPI_TalonFX(HOOD_MOTOR_ID);
    private final WPI_TalonFX flywheel1Motor = new WPI_TalonFX(FLYWHEEL_MOTOR1_ID);
    private final WPI_TalonFX flywheel2Motor = new WPI_TalonFX(FLYWHEEL_MOTOR2_ID);

    private final MagneticLimitSwitch outerLimSwitch = new MagneticLimitSwitch(OUTER_LIM_SWITCH_ID);
    private final MagneticLimitSwitch centerLimitSwitch = new MagneticLimitSwitch(CENTER_LIM_SWITCH_ID);

    private final Limelight limelight = Limelight.getInstance();

    private final InterpLUT RPMLUT = new InterpLUT();
    private final InterpLUT hoodAngleLUT = new InterpLUT();
    private final InterpLUT ejectionRPMLUT = new InterpLUT();

    // private final SimpleMotorFeedforward flywheelFeedForward = new SimpleMotorFeedforward(FLYWHEEL_KS, FLYWHEEL_KV);
    // private final PIDController flywheelPID = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);

    private double flywheelTarget = 0;
    private double hoodTarget = 0;

    // private final SimpleMotorFeedforward rotaryFeedForward = new SimpleMotorFeedforward(ROTARY_KS, ROTARY_KV);
    // private final ProfiledPIDController rotaryPID = new ProfiledPIDController(ROTARY_KP, ROTARY_KI, ROTARY_KD, NORMAL_ROTARY_CONSTRAINTS);
    private boolean rotaryControlLoopEnabled = true;
    private double prevRotaryAngle;
    private RotarySpeed rotarySpeed = Normal;

    // private final SimpleMotorFeedforward hoodFeedForward = new SimpleMotorFeedforward(HOOD_KS, HOOD_KV);
    // private final ProfiledPIDController hoodPID = new ProfiledPIDController(HOOD_KP, HOOD_KI, HOOD_KD,
            // new TrapezoidProfile.Constraints(HOOD_MAX_VEL, HOOD_MAX_ACCEL));
    private boolean hoodControlLoopEnabled = true;

    private boolean hoodOverextended = false;
    private boolean rotaryOverextended = false;

    private boolean trustResetting = true; //Whether or not the bot should just assume everything is reset properly
    private double rotarySetpoint = 0;

    public Turret() {
        super(TurretState.class);

        MotorConfiguration.configureBasicMotor(hoodMotor);
        MotorConfiguration.configureMotionMagicMotor(hoodMotor, hoodGains, hoodDegreesToTicks(HOOD_MAX_VEL) * 10, hoodDegreesToTicks(HOOD_MAX_ACCEL) * 10);

        MotorConfiguration.configureBasicMotor(flywheel1Motor, false, true, true);
        MotorConfiguration.configureBasicMotor(flywheel2Motor, false, false, true);

        flywheel2Motor.follow(flywheel1Motor);
        flywheel2Motor.setInverted(OpposeMaster);

        //Set the tolerance for the PID loop to say it is not busy
        // hoodPID.setTolerance(HOOD_TOLERANCE);

        
        /* Factory default hardware to prevent unexpected behavior */
		rotaryMotor.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		rotaryMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.Turret.rotarykPIDLoopIdx,
				Constants.Turret.rotarykTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
        rotaryMotor.configNeutralDeadband(0.001, Constants.Turret.rotarykTimeoutMs);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		rotaryMotor.setSensorPhase(false);
		rotaryMotor.setInverted(true);
        rotaryMotor.configSupplyCurrentLimit(Constants.CURRENT_LIMIT);
		
		/* Set relevant frame periods to be at least as fast as periodic rate */
		rotaryMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.Turret.rotarykTimeoutMs);
		rotaryMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.Turret.rotarykTimeoutMs);

		/* Set the peak and nominal outputs */
		rotaryMotor.configNominalOutputForward(0, Constants.Turret.rotarykTimeoutMs);
		rotaryMotor.configNominalOutputReverse(0, Constants.Turret.rotarykTimeoutMs);
		rotaryMotor.configPeakOutputForward(1, Constants.Turret.rotarykTimeoutMs);
		rotaryMotor.configPeakOutputReverse(-1, Constants.Turret.rotarykTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		rotaryMotor.selectProfileSlot(Constants.Turret.rotarykSlotIdx, Constants.Turret.rotarykPIDLoopIdx);
		rotaryMotor.config_kF(Constants.Turret.rotarykSlotIdx, Constants.Turret.rotaryGains.kF, Constants.Turret.rotarykTimeoutMs);
		rotaryMotor.config_kP(Constants.Turret.rotarykSlotIdx, Constants.Turret.rotaryGains.kP, Constants.Turret.rotarykTimeoutMs);
		rotaryMotor.config_kI(Constants.Turret.rotarykSlotIdx, Constants.Turret.rotaryGains.kI, Constants.Turret.rotarykTimeoutMs);
		rotaryMotor.config_kD(Constants.Turret.rotarykSlotIdx, Constants.Turret.rotaryGains.kD, Constants.Turret.rotarykTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		rotaryMotor.configMotionCruiseVelocity(rotaryDegreesToTicks(ROTARY_MAX_VEL) * 10, Constants.Turret.rotarykTimeoutMs);
		rotaryMotor.configMotionAcceleration(rotaryDegreesToTicks(ROTARY_MAX_ACCEL) * 10, Constants.Turret.rotarykTimeoutMs);

        //Flywheel motor set up
        MotorConfiguration.configureVelocityMotor(flywheel1Motor, flywheelGains);

        //Hood setup
        

		/* Zero the sensor once on robot boot up */
		// rotaryMotor.setSelectedSensorPosition(0, Constants.Turret.kPIDLoopIdx, Constants.Turret.kTimeoutMs);

        //Setup lookup tables for velocity and angle control
        RPMLUT.add(Units.inchesToMeters(0), 2000); //68 in.
        RPMLUT.add(Units.inchesToMeters(72), 2000); //68 in.
        RPMLUT.add(Units.inchesToMeters(126.5), 2000); //126.5 in
        RPMLUT.add(Units.inchesToMeters(173), 2100); //173 in
        RPMLUT.add(5.1, 2350);
        RPMLUT.add(6.1, 2600);
        RPMLUT.add(1000, 2350);
        RPMLUT.createLUT();

        hoodAngleLUT.add(Units.inchesToMeters(0), 3); //68 in.
        hoodAngleLUT.add(Units.inchesToMeters(72), 9); //68 in.
        hoodAngleLUT.add(Units.inchesToMeters(126.5), 18); //126.5 in.
        hoodAngleLUT.add(Units.inchesToMeters(173), 23); //173 in.
        hoodAngleLUT.add(5.1, 25.5); //173 in.
        hoodAngleLUT.add(6.1, 29.5); //173 in.
        hoodAngleLUT.add(1000, 29.5); 
        hoodAngleLUT.createLUT();

        ejectionRPMLUT.add(0, 0);
        ejectionRPMLUT.add(1000, 25);
        ejectionRPMLUT.createLUT();;

        getRotaryAngle = () -> Rotation2d.fromDegrees(getRotaryAngle());
        getLimelightXOffsetAngle = () -> Rotation2d.fromDegrees(getLimelightXOffsetDegrees());

        addDetermination(Undetermined, Idle, new InstantCommand(() -> {
            // resetHoodPos(0);
            resetHoodPID();
            setHoodTargetAngle(2);
            resetRotaryPos(0);
            setRotarySpeed(Normal);
            turnOffLimelight();
            setFlywheelTargetRPM(0);
        }));

        setContinuousCommand(Idle, new InstantCommand(() -> {
            if(!trustResetting) {
                requestTransition(Resetting);
            }
        }));

        addCommutativeTransition(Idle, ActiveTracking, new InstantCommand((this::turnOnLimelight)), new InstantCommand((this::turnOffLimelight)));

        AtomicBoolean lockedIn = new AtomicBoolean(false);
        setContinuousCommand(ActiveTracking, new ActiveTracking(this, RPMLUT, hoodAngleLUT, (value) -> lockedIn.set(value)));

        addFlagState(ActiveTracking, LockedIn, () -> lockedIn.get());

        addTransition(Idle, Resetting, new InstantCommand());
        setContinuousCommand(Resetting, new DetermineTurretState(this, () -> trustResetting));
        addTransition(Resetting, Idle, new InstantCommand());

        addTransition(Idle, ClimbLock, new InstantCommand(() -> setRotaryTargetAngle(0)));

        //Ejecting logic
        addTransition(Idle, Ejecting, new InstantCommand(() -> setHoodTargetAngle(HOOD_MAX_ANGLE)));
        setContinuousCommand(Ejecting, new AutoEjecting(this, ejectionRPMLUT, Constants.SwerveDrivetrain.getOdoPose));
        addTransition(Ejecting, Idle, new InstantCommand(() -> {
            setFlywheelTargetRPM(0);
            setHoodTargetAngle(HOOD_MIN_ANGLE);
        }));

        addTransition(ActiveTracking, Ejecting, new InstantCommand(() -> {
            turnOffLimelight();
            setHoodTargetAngle(HOOD_MAX_ANGLE);
        }));
        addTransition(Ejecting, ActiveTracking, new InstantCommand(() -> {
            turnOnLimelight();
            setHoodTargetAngle(HOOD_MIN_ANGLE);
        }));
    }

    public void setFlywheelTargetRPM(double rpm) {
        if(rpm != 0) {
            flywheel1Motor.set(TalonFXControlMode.Velocity, RPMToFlywheelTicks(rpm));
        } else {
            flywheel1Motor.set(ControlMode.PercentOutput, 0);
        }
        flywheelTarget = rpm;
    }

    public void setRotaryTargetAngle(double degrees) {
        double clippedDegrees = ROTARY_RANGE.clipToRange(degrees);
        if(clippedDegrees != degrees) rotaryOverextended = true;
        else if (rotaryOverextended) rotaryOverextended = false;

        rotarySetpoint = clippedDegrees;

        // rotaryMotor.set(TalonFXControlMode.MotionMagic, degreesToTicks(clippedDegrees));
    }

    public void setHoodTargetAngle(double degrees) {
        double clippedDegrees = HOOD_RANGE.clipToRange(degrees);
        if(clippedDegrees != degrees) hoodOverextended = true;
        else if (hoodOverextended) hoodOverextended = false;

        hoodMotor.set(TalonFXControlMode.MotionMagic, hoodDegreesToTicks(degrees));
        hoodTarget = degrees;
    }

    @Override
    public void onEnable() {
        resetAllPIDs();
        setHoodTargetAngle(2);
    }

    @Override
    public void onDisable() {
        runInstantaneousTransition(
                Idle,
                () -> {
                    setHoodTargetAngle(getHoodAngle());
                    setFlywheelTargetRPM(0);
                    setRotaryTargetAngle(getRotaryAngle());
                    turnOffLimelight();
                }
        );
    }

    public void resetAllPIDs() {
        resetHoodPID();
        resetHoodPID();
    }

    public void resetHoodPID() {
        // hoodPID.reset(getHoodAngle(), getHoodVelo());
    }

    @Override
    public void update() {
        //Hood calculations
        // updateHood();

        prevRotaryAngle = getRotaryAngle();
    }


    // private void updateHood() {
    //     if(hoodControlLoopEnabled) {
    //         double hoodFFOutput = hoodFeedForward.calculate(hoodPID.getSetpoint().velocity);
    //         double hoodPIDOutput = hoodPID.calculate(getHoodAngle());

    //         double hoodSum = hoodFFOutput + hoodPIDOutput;
    //         if(getHoodAngle() <= HOOD_MIN_ANGLE && hoodSum < 0) hoodSum = 0;
    //         else if(getHoodAngle() >= HOOD_MAX_ANGLE && hoodSum > 0) hoodSum = 0;

    //         hoodMotor.setVoltage(hoodSum);
    //     }
    // }

    /**
     * @return RPM
     */
    public double getFlywheelRPM() {
        return
            flywheel1Motor.getSelectedSensorVelocity() * // Ticks/100ms
            10 / // Ticks/sec
            2048 * // Rotations/sec
            60 //Rotations/min
        ;
    }

    /**
     * @return ticks / 100 ms
     */
    public double RPMToFlywheelTicks(double rpm) {
        return rpm
            / 60 //r/s
            * 2048  // t/s
            / 10  // t/100ms
        ;
    }

    /**
     * @return deg
     */
    public double getRotaryAngle() {
        return
            rotaryMotor.getSelectedSensorPosition() //Ticks
            / 2048 * //Rotations
            (1.0/3.0) * //Gear ratio on versaplanetary
            (40.0 / 140) * //Rotations
            360 //deg
        ;
    }

    /**
     * @return Deg/sec
     */
    public double getRotaryVelo() {
        return
            rotaryMotor.getSelectedSensorVelocity() * //Ticks/100ms
                    10  //Ticks/sec
                    /2048 * //Rotations/sec
                    (1.0 / 3.0) * //Gear ratio on versaplanetary
                    (40.0 / 140) //Rotations
                    * 360 //Degrees
        ;
    }

    private double rotaryDegreesToTicks(double degrees) {
        return degrees
        / 360.0 
        * (140.0 / 40.0)
        * (3.0 / 1.0)
        * 2048;
    }

    /**
     * @return deg
     */
    public double getHoodAngle() {
        return
                hoodMotor.getSelectedSensorPosition() //Ticks
                / 10 //Gearbox
                / 2048 * //Rotations
                (18.0 / 485.0) //Output rotations
                * 360 //Output degrees
        ;
    }

    public double hoodDegreesToTicks(double degrees) {
        return degrees //deg
        / 360 //output rotation
        * (485.0 / 18.0) //input rotations
        * 10 
        * 2048; //input degrees
    }

    /**
     * @return deg/sec
     */
    public double getHoodVelo() {
        return
                hoodMotor.getSelectedSensorVelocity() //Ticks/100ms
                        * 10 //Ticks/sec
                        /10 //Gearbox
                        /2048 * //Rotations/sec
                        (18.0 / 485.0) //Output rot/sec
                        * 360 //Output degrees
                ;
    }

    public boolean isOuterLimPressed() {return outerLimSwitch.isTripped();}
    public boolean isCenterLimPressed() {return centerLimitSwitch.isTripped();}
    public double getPrevRotaryAngle() {return prevRotaryAngle;}

    public boolean isRotaryBusy() {return Math.abs(getRotaryAngle() - rotarySetpoint) <= ROTARY_TOLERANCE;}
    public boolean isFlywheelBusy() {
        return Math.abs(getFlywheelRPM() - getFlywheelTarget()) > FLYWHEEL_TOLERANCE;
    }
    public boolean isHoodBusy() {return Math.abs(getHoodAngle() - getHoodTarget()) > HOOD_TOLERANCE;}

    public boolean isRotaryOverextended() {return rotaryOverextended;}
    public boolean isHoodOverextended() {return hoodOverextended;}

    public void setRotarySpeed(RotarySpeed speed) {
        rotarySpeed = speed;

        TrapezoidProfile.Constraints c = speed == Normal ? NORMAL_ROTARY_CONSTRAINTS : SEARCH_ROTARY_CONSTRAINTS;
        
        rotaryMotor.configMotionCruiseVelocity(rotaryDegreesToTicks(c.maxVelocity), Constants.Turret.rotarykTimeoutMs);
		rotaryMotor.configMotionAcceleration(rotaryDegreesToTicks(c.maxAcceleration), Constants.Turret.rotarykTimeoutMs);
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
    public double getFlywheelTarget() {return flywheelTarget;}
    public double getFlywheelError() {return abs(getFlywheelTarget()-getFlywheelRPM());}
    public double getHoodTarget() {return hoodTarget;}
    public double getHoodError() {return abs(getHoodTarget() - getHoodAngle());}
    public double getRotaryTarget() {return rotarySetpoint;}
    public double getRotaryError() {return abs(getRotaryTarget() - getRotaryAngle());}

    //Limelight access methods
    public void turnOnLimelight() {limelight.setOn();}
    public void turnOffLimelight() {limelight.setOff();}
    public boolean doesLimelightHaveTarget() {return limelight.hasTarget();}
    public double getLimelightXOffsetDegrees() {return limelight.getXOffset().getDegrees();}
    public double getLimelightYOffsetDegrees() {return limelight.getYOffset().getDegrees();}

    @Override
    public String getName() {return "turret";}

    @Override
    public void additionalSendableData(SendableBuilder builder) {
        builder.addDoubleProperty("flywheel velo", this::getFlywheelRPM, null);
        builder.addDoubleProperty("flywheel target", this::getFlywheelTarget, null);
        builder.addDoubleProperty("flywheel error", this::getFlywheelError, null);
        builder.addBooleanProperty("flywheel busy", this::isFlywheelBusy, null);
        // builder.addDoubleProperty("flywheel 1 voltage", flywheel1Motor::getMotorOutputVoltage, null);
        // builder.addDoubleProperty("flywheel 2 voltage", flywheel2Motor::getMotorOutputVoltage, null);
        // builder.addDoubleProperty("rotary degrees", this::getRotaryAngle, null);
        // builder.addDoubleProperty("rotary velo", this::getRotaryVelo, null);
        // builder.addDoubleProperty("rotary target", this::getRotaryTarget, null);
        // builder.addDoubleProperty("rotary error", this::getRotaryError, null);
        // builder.addBooleanProperty("rotary busy", this::isRotaryBusy, null);
        // builder.addDoubleProperty("rotary voltage", () -> rotaryMotor.getMotorOutputVoltage(), null);
        builder.addDoubleProperty("hood degrees", this::getHoodAngle, null);
        builder.addDoubleProperty("hood raw angle", hoodMotor::getSelectedSensorPosition, null);
        builder.addDoubleProperty("hood raw target", () -> hoodDegreesToTicks(getHoodTarget()), null);
        builder.addDoubleProperty("hood velo", this::getHoodVelo, null);
        // builder.addDoubleProperty("hood target velo", () -> hoodPID.getSetpoint().velocity, null);
        builder.addDoubleProperty("hood target", this::getHoodTarget, null);
        builder.addDoubleProperty("hood error", this::getHoodError, null);
        builder.addBooleanProperty("hood busy", this::isHoodBusy, null);
        // builder.addBooleanProperty("sensor 1 pressed", this::isOuterLimPressed, null);
        // builder.addBooleanProperty("sensor 2 pressed", this::isCenterLimPressed, null);

    }

    public enum TurretState {
        Undetermined, Idle, Resetting, Ejecting, ActiveTracking, LockedIn, ClimbLock
    }

    public enum RotarySpeed {
        Normal, Search
    }

    public void setFlywheelPower(double power) {
        flywheel1Motor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void setHoodPower(double power) {
        hoodMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void setRotaryPower(double power) {
        rotaryMotor.set(TalonFXControlMode.PercentOutput, power);
    }
}
