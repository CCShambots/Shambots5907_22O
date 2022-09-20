package frc.robot.util;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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

        turnMotor.configSelectedFeedbackSensor(
                TalonFXFeedbackDevice.IntegratedSensor,
                /*TODO: add constant*/0,
                /*TODO: add constant*/30
        );

        turnMotor.configNeutralDeadband(0.001/*TODO: change? or add constant*/);

        //TODO: DO THE REST OF MOTOR CONFIG AS SEEN IN TURRET BRANCH

        targetState = new SwerveModuleState(0, getTurnAngle());
    }

    private void initSelectedTurnEncoder() {
        turnMotor.setSelectedSensorPosition(degreesToTicks(turnEncoder.getAbsolutePosition() - encoderOffset));
    }

    private double degreesToTicks(double degrees) {
        //assuming degrees is always positive
        return ((degrees % 360) * (4096.0/360.0));
    }

    private double ticksToDegrees(double ticks) {
        //prob doesn't work idk :)
        double out = Math.copySign(ticks, Math.abs(ticks) % 2048);
        return ticks >= 0 ?
                (out * (360.0/2048.0)) :
                ((out + 2048) * (360.0/2048.0));
    }

    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
        targetState = optimizedState;
    }

    public Rotation2d getTurnAngle(){
        return Rotation2d.fromDegrees(ticksToDegrees(turnMotor.getSelectedSensorPosition()));
    }

    public double getDriveMotorRate(){
        return driveMotor.getSelectedSensorVelocity()
                * 10.0          // convert sensor ticks per 100ms to sensor ticks per second
                * (1 / 2048.0)  // convert sensor ticks to revolutions
                * (1 / Constants.SwerveModule.DRIVE_RATIO)  // convert motor revolutions to wheel revolutions
                * (2 * Math.PI * Constants.SwerveModule.WHEEL_RADIUS);   // convert wheel revolutions to linear distance
    } 

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveMotorRate(), getTurnAngle());
    }

    public void run() {
        //BOO WPI PID >:( (sorry wpi)
        /*double turnPIDOutput = turnPIDController.calculate(getTurnAngle().getRadians(), targetState.angle.getRadians());
        double drivePIDOutput = drivePIDController.calculate(getDriveMotorRate(), targetState.speedMetersPerSecond);

        double turnFFOutput = turnFeedforwardController.calculate(turnPIDController.getSetpoint().velocity);
        double driveFFOutput = driveFeedforwardController.calculate(targetState.speedMetersPerSecond);*/

        //haha voltages
        /*turnMotor.setVoltage(turnPIDOutput + turnFFOutput);
        driveMotor.setVoltage(drivePIDOutput + driveFFOutput);*/

        //remvoe?
        // SmartDashboard.putNumber(this.moduleName + " turn PID output", turnPIDOutput);
        // SmartDashboard.putNumber(this.moduleName + " turn FF output", turnFFOutput);
        // SmartDashboard.putNumber(this.moduleName + " drive PID output", drivePIDOutput);
        // SmartDashboard.putNumber(this.moduleName + " drive FF output", driveFFOutput);


    }

    public void stop() {
        targetState = new SwerveModuleState(0.0, getTurnAngle());
    }

    public void resetControlLoops() {
        stop();
        /*drivePIDController.reset();
        turnPIDController.reset(getTurnAngle().getRadians());*/
        //TODO: idk if this works
        turnMotor.set(0);
        driveMotor.set(0);
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
