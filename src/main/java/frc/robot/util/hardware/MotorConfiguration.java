package frc.robot.util.hardware;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class MotorConfiguration {

    public static void configureBasicMotor(WPI_TalonFX motor, boolean braked, boolean reversed, boolean configFactoryDefault) {
        if(configFactoryDefault) motor.configFactoryDefault();
        motor.configSupplyCurrentLimit(Constants.CURRENT_LIMIT);
        motor.setInverted(reversed);
        motor.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Default configuration for a motor. Assumes that the motor should be braked, but not reversed
     * @param motor Talon to configure
     */
    public static void configureBasicMotor(WPI_TalonFX motor) {
        configureBasicMotor(motor, true, false, true);}

    /**
     * Default configuration for a motor. Assumes that the motor should be braked, but not reversed
     * @param motor Talon to configure
     */
    public static void configureBasicMotor(WPI_TalonFX motor, boolean configFactoryDefault) {
        configureBasicMotor(motor, true, false, configFactoryDefault);}


    public static void configureVelocityMotor(WPI_TalonFX motor, Gains gains) {configureVelocityMotor(motor, gains, 0, 0, Constants.CURRENT_LIMIT, 30);}

    public static void configureVelocityMotor(WPI_TalonFX motor, Gains gains, int kSlotIdx, int kPIDLoopIdx, SupplyCurrentLimitConfiguration currentLimit, int kTimeoutMs) {
        motor.setSensorPhase(false);
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

        motor.configSupplyCurrentLimit(currentLimit);

        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);

        motor.configNominalOutputForward(0, kTimeoutMs);
        motor.configNominalOutputReverse(0, kTimeoutMs);
        motor.configPeakOutputForward(1, kTimeoutMs);
        motor.configPeakOutputReverse(-1, kTimeoutMs);

        motor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
        motor.config_kF(kSlotIdx, gains.kF, kTimeoutMs);
        motor.config_kP(kSlotIdx, gains.kP, kTimeoutMs);
        motor.config_kI(kSlotIdx, gains.kI, kTimeoutMs);
        motor.config_kD(kSlotIdx, gains.kD, kTimeoutMs);
    }

    public static void configureMotionMagicMotor(WPI_TalonFX motor, Gains gains, double maxVelo, double maxAccel) { configureMotionMagicMotor(motor, gains, 0, 0, maxVelo, maxAccel, Constants.CURRENT_LIMIT, 30);}

    public static void configureMotionMagicMotor(WPI_TalonFX motor, Gains gains, int kSlotIdx, int kPIDLoopIdx, double maxSpeed, double maxAccel, SupplyCurrentLimitConfiguration currentLimit, int kTimeoutMs) {
        motor.setSensorPhase(false);
        motor.configSupplyCurrentLimit(currentLimit);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

        /* Set the peak and nominal outputs */
        motor.configNominalOutputForward(0, kTimeoutMs);
        motor.configNominalOutputReverse(0, kTimeoutMs);
        motor.configPeakOutputForward(1, kTimeoutMs);
        motor.configPeakOutputReverse(-1, kTimeoutMs);

        /* Set Motion Magic gains in slot0 - see documentation */
        motor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
        motor.config_kF(kSlotIdx, gains.kF, kTimeoutMs);
        motor.config_kP(kSlotIdx, gains.kP, kTimeoutMs);
        motor.config_kI(kSlotIdx, gains.kI, kTimeoutMs);
        motor.config_kD(kSlotIdx, gains.kD, kTimeoutMs);

        /* Set acceleration and cruise velocity - see documentation */
        motor.configMotionCruiseVelocity(
                Math.toDegrees(maxSpeed),
                kTimeoutMs
        );

        motor.configMotionAcceleration(
                Math.toDegrees(maxAccel),
                kTimeoutMs
        );

        motor.configNeutralDeadband(0.001);

        motor.configSelectedFeedbackSensor(
                TalonFXFeedbackDevice.IntegratedSensor,
                kPIDLoopIdx,
                kTimeoutMs
        );
    }


}
