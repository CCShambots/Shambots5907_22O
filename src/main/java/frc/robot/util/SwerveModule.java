package frc.robot.util;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;

public class SwerveModule implements Sendable{

    private final String moduleName;

    private final WPI_TalonFX turnMotor;
    private final WPI_TalonFX driveMotor;

    private final CANCoder turnEncoder;
    private double encoderOffset;

    private SwerveModuleState targetState;

    private double targetEncoderPos;

    public SwerveModule(String name, int turnID, int driveID, int encoderID, double encoderOffset, boolean reverseDriveMotor, boolean reverseTurnEncoder) {
        this.moduleName = name;
        turnMotor = new WPI_TalonFX(turnID, "Drivetrain");
        turnMotor.configFactoryDefault();
        
        driveMotor = new WPI_TalonFX(driveID, "Drivetrain");
        driveMotor.configFactoryDefault();
        if(reverseDriveMotor) driveMotor.setInverted(true);
        
        this.turnEncoder = new CANCoder(encoderID);
        turnEncoder.configFactoryDefault();

        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnEncoder.configSensorDirection(false);

        this.encoderOffset = encoderOffset;

        initTurnMotor();
        initDriveMotor();

        setDesiredState(
            new SwerveModuleState(0, getTurnAngle())
        );
    }   
        
    private void initTurnMotor() {
        turnMotor.setSensorPhase(false);
        turnMotor.configSupplyCurrentLimit(Constants.CURRENT_LIMIT);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        turnMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.SwerveModule.kTimeoutMs);
        turnMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.SwerveModule.kTimeoutMs);

        /* Set the peak and nominal outputs */
        turnMotor.configNominalOutputForward(0, Constants.SwerveModule.kTimeoutMs);
        turnMotor.configNominalOutputReverse(0, Constants.SwerveModule.kTimeoutMs);
        turnMotor.configPeakOutputForward(1, Constants.SwerveModule.kTimeoutMs);
        turnMotor.configPeakOutputReverse(-1, Constants.SwerveModule.kTimeoutMs);

        /* Set Motion Magic gains in slot0 - see documentation */
        turnMotor.selectProfileSlot(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.kPIDLoopIdx);
        turnMotor.config_kF(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.turnGains.kF, Constants.SwerveModule.kTimeoutMs);
        turnMotor.config_kP(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.turnGains.kP, Constants.SwerveModule.kTimeoutMs);
        turnMotor.config_kI(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.turnGains.kI, Constants.SwerveModule.kTimeoutMs);
        turnMotor.config_kD(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.turnGains.kD, Constants.SwerveModule.kTimeoutMs);

        /* Set acceleration and cruise velocity - see documentation */
        turnMotor.configMotionCruiseVelocity(
                Math.toDegrees(Constants.SwerveModule.MAX_TURN_SPEED),
                Constants.SwerveModule.kTimeoutMs
        );

        turnMotor.configMotionAcceleration(
                Math.toDegrees(Constants.SwerveModule.MAX_TURN_ACCEL),
                Constants.SwerveModule.kTimeoutMs
        );

        turnMotor.configNeutralDeadband(0.001);

        turnMotor.configSelectedFeedbackSensor(
                TalonFXFeedbackDevice.IntegratedSensor,
                /*TODO: add constant*/0,
                /*TODO: add constant*/30
        );

        turnMotor.configSelectedFeedbackCoefficient(
                Constants.SwerveModule.TURN_SENSOR_RATIO * (360.0/2048.0)
        );
        turnMotor.setSelectedSensorPosition(normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset));
    }

    private void initDriveMotor() {
        driveMotor.setSensorPhase(false);
        
        driveMotor.configSupplyCurrentLimit(Constants.CURRENT_LIMIT);
        
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.SwerveModule.kTimeoutMs);

        driveMotor.configNominalOutputForward(0, Constants.SwerveModule.kTimeoutMs);
        driveMotor.configNominalOutputReverse(0, Constants.SwerveModule.kTimeoutMs);
        driveMotor.configPeakOutputForward(1, Constants.SwerveModule.kTimeoutMs);
        driveMotor.configPeakOutputReverse(-1, Constants.SwerveModule.kTimeoutMs);

        turnMotor.selectProfileSlot(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.kPIDLoopIdx);
        turnMotor.config_kF(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.driveGains.kF, Constants.SwerveModule.kTimeoutMs);
        turnMotor.config_kP(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.driveGains.kP, Constants.SwerveModule.kTimeoutMs);
        turnMotor.config_kI(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.driveGains.kI, Constants.SwerveModule.kTimeoutMs);
        turnMotor.config_kD(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.driveGains.kD, Constants.SwerveModule.kTimeoutMs);

        driveMotor.configMotionCruiseVelocity(
                //TODO: change coefficient and this to m/s
                Math.toDegrees(Constants.SwerveModule.MAX_DRIVE_SPEED),
                Constants.SwerveModule.kTimeoutMs
        );

        driveMotor.configMotionAcceleration(
                //TODO: change coefficient and this to m/s
                Math.toDegrees(Constants.SwerveModule.MAX_DRIVE_ACCEL),
                Constants.SwerveModule.kTimeoutMs
        );

        driveMotor.configSelectedFeedbackCoefficient(
                //TODO: idk if this math is correct
                (1 / 2048.0)
                * (1 / Constants.SwerveModule.DRIVE_RATIO)
                * (2 * Math.PI * Constants.SwerveModule.WHEEL_RADIUS)      
        );
    }



    private double normalizeDegrees(double degrees) {
        double out = Math.copySign(Math.abs(degrees) % 180, degrees);
        return out;
    }

    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
        targetState = optimizedState;
        double turnPos = turnMotor.getSelectedSensorPosition();
        targetEncoderPos = turnPos + (targetState.angle.getDegrees() - normalizeDegrees(turnPos));

        turnMotor.set(
                ControlMode.MotionMagic,
                targetEncoderPos,
                DemandType.ArbitraryFeedForward,
                Constants.SwerveModule.KS_TURN/12
        );

    }

    public Rotation2d getTurnAngle(){
        return Rotation2d.fromDegrees(normalizeDegrees(turnMotor.getSelectedSensorPosition()));
    }

    public double getDriveMotorRate(){
        return driveMotor.getSelectedSensorVelocity() * 10.0;
    } 

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveMotorRate(), getTurnAngle());
    }

    public void run() {
        
        driveMotor.set(
                ControlMode.Velocity,
                targetState.speedMetersPerSecond/10,
                DemandType.ArbitraryFeedForward,
                Constants.SwerveModule.KS_DRIVE/12
        );
    }

    public void stop() {
        setDesiredState(new SwerveModuleState(0.0, getTurnAngle()));
    }

    public String getModuleName() {
        return moduleName;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");

        builder.addDoubleProperty("Angle", () -> getTurnAngle().getDegrees(), null);
        builder.addDoubleProperty("Target Angle", () -> targetState.angle.getDegrees(), null);
        builder.addDoubleProperty("Velocity", () -> getDriveMotorRate(), null);
        builder.addDoubleProperty("Target Velocity", () -> targetState.speedMetersPerSecond, null);
        builder.addDoubleProperty("Encoder offset", () -> encoderOffset, null);
        //TODO: add these but for ctre pid
        //builder.addDoubleProperty("Target turn velo", () -> turnPIDController.getSetpoint().velocity, null);
        //builder.addDoubleProperty("Measuerd turn velo", () -> reverseTurnEncoder ? -1 : 1 * turnEncoder.getVelocity(), null);
        
    }
    
}
