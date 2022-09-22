package frc.robot.util;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;

public class SwerveModule implements Sendable{

    private final String moduleName;

    private final WPI_TalonFX turnMotor;
    private final WPI_TalonFX driveMotor;

    private final CANCoder turnEncoder;
    private double encoderOffset;

    private SwerveModuleState targetState;

    public SwerveModule(String name, int turnID, int driveID, int encoderID, double encoderOffset, boolean reverseDriveMotor, boolean reverseTurnEncoder) {
        this.moduleName = name;
        turnMotor = new WPI_TalonFX(turnID);
        turnMotor.configFactoryDefault();
        
        driveMotor = new WPI_TalonFX(driveID);
        driveMotor.configFactoryDefault();
        if(reverseDriveMotor) driveMotor.setInverted(true);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5);
        
        this.turnEncoder = new CANCoder(encoderID);
        turnEncoder.configFactoryDefault();
        // turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 5);

        //can be removed?
        // vv
        double radiansCoefficient = (2.0 * Math.PI) / 4096.0;
        turnEncoder.configFeedbackCoefficient(radiansCoefficient, "rad", SensorTimeBase.PerSecond);
        // ^^

        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnEncoder.configSensorDirection(false);

        this.encoderOffset = encoderOffset + 180;

        initSelectedTurnEncoder();

        driveMotor.configSelectedFeedbackCoefficient(
                //TODO: idk if this math is correct
                (1 / 2048.0)
                * (1 / Constants.SwerveModule.DRIVE_RATIO)
                * (2 * Math.PI * Constants.SwerveModule.WHEEL_RADIUS)
        );

        turnMotor.configSelectedFeedbackSensor(
                TalonFXFeedbackDevice.IntegratedSensor,
                /*TODO: add constant*/0,
                /*TODO: add constant*/30
        );

        initTurnPID();

        turnMotor.configNeutralDeadband(0.001/*TODO: change? or add constant*/);

        //TODO: DO THE REST OF MOTOR CONFIG AS SEEN IN SwerveModule BRANCH

        targetState = new SwerveModuleState(0, getTurnAngle());
    }

    private void initDrivePID() {
        driveMotor.setSensorPhase(false);

        driveMotor.configSupplyCurrentLimit(Constants.CURRENT_LIMIT);

        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.SwerveModule.kTimeoutMs);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.SwerveModule.kTimeoutMs);

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
    }

    private void initTurnPID() {
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

        /* Set acceleration and vcruise velocity - see documentation */
        turnMotor.configMotionCruiseVelocity(
                Math.toDegrees(Constants.SwerveModule.MAX_TURN_SPEED),
                Constants.SwerveModule.kTimeoutMs
        );

        turnMotor.configMotionAcceleration(
                Math.toDegrees(Constants.SwerveModule.MAX_TURN_ACCEL),
                Constants.SwerveModule.kTimeoutMs
        );
    }

    private void initSelectedTurnEncoder() {
        turnMotor.configSelectedFeedbackCoefficient(
                Constants.SwerveModule.TURN_SENSOR_RATIO * (360.0/2048.0)
        );
        turnMotor.setSelectedSensorPosition(normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset));
    }

    private double normalizeDegrees(double degrees) {
        double out = Math.copySign(degrees, Math.abs(degrees) % 360);
        return out >= 0 ?
                out :
                out + 360;
    }

    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
        targetState = optimizedState;
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
        double turnPos = turnMotor.getSelectedSensorPosition();
        turnMotor.set(
                ControlMode.MotionMagic,
                turnPos + (targetState.angle.getDegrees() - normalizeDegrees(turnPos)),
                DemandType.ArbitraryFeedForward,
                Constants.SwerveModule.KS_TURN/12
        );

        driveMotor.set(
                ControlMode.Velocity,
                targetState.speedMetersPerSecond/10,
                DemandType.ArbitraryFeedForward,
                Constants.SwerveModule.KS_DRIVE/12
        );
    }

    public void stop() {
        targetState = new SwerveModuleState(0.0, getTurnAngle());
    }

    public void resetControlLoops() {
        stop();
        /*drivePIDController.reset();
        turnPIDController.reset(getTurnAngle().getRadians());*/
        //TODO: idk if this works
        turnMotor.set(ControlMode.PercentOutput, 0);
        driveMotor.set(ControlMode.PercentOutput, 0);
        stop();
    }

    public void setEncoderOffset(){
        double newOffset = getTurnAngle().getRadians() + encoderOffset;
        if (newOffset > Math.PI){ newOffset -= Math.PI; }
        if (newOffset < -Math.PI){ newOffset += Math.PI; }
        Preferences.setDouble(moduleName + "-encoder-offset", newOffset);
        encoderOffset = newOffset;
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
