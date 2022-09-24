// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.hardware.Gains;

import java.util.function.UnaryOperator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1); //enable these limits, current limit, trigger threshold, trigger threshold time


    public static final class SwerveModule{
        // Physical Constants
        public static final double WHEEL_RADIUS = 0.0508;
        // Use this ratio to convert from CANCoder absolute position to radians
        public static final double TURN_SENSOR_RATIO = 1.0/(150.0/7.0);

        // Drive motor PID controller coefficients
        public static final double P_DRIVE = 0.2;
        public static final double I_DRIVE = 0;
        public static final double D_DRIVE = 0;
        public static final double KS_DRIVE = .25;
        public static final double KV_DRIVE = 0.04;

        // Use this ratio to convert from Falcon angular velocity to wheel angular velocity
        public static final double DRIVE_RATIO = 8.14;

        // Turn motor PID controller coefficients
        // using a trapezoidal profile
        public static final double P_TURN = 9; //2
        public static final double I_TURN = 0;
        public static final double D_TURN = 0; //0.1
        public static final double KS_TURN = 0.75; //0.05
        public static final double KV_TURN = 1; //0.15
        public static final double MAX_TURN_SPEED = 50; // Rad/S //50
        public static final double MAX_TURN_ACCEL = 400; // Rad/S^2
        public static final double MAX_DRIVE_SPEED = 0;//TODO: CHANGE THIS
        public static final double MAX_DRIVE_ACCEL = 0;//TODO: CHANGE THIS

        public static final Gains turnGains = new Gains(
                P_TURN,
                I_TURN,
                D_TURN,
                KV_TURN,
                0, //TODO: change this if needed
                20 //TODO: change this
        );

        public static final Gains driveGains = new Gains(
                P_DRIVE,
                I_DRIVE,
                D_DRIVE,
                KV_DRIVE,
                0, //TODO: change this if needed,
                20 //TODO: change this
        );

        public static final int kTimeoutMs = 30;//TODO: change these if needed vv
        public static final int kPIDLoopIdx = 0;
        public static final int kSlotIdx = 0;
    }

    public static final class SwerveDrivetrain{

        // Distance between centers of right and left wheels on robot in meters
        public static final double TRACK_WIDTH = 0.62865;
        // Distance between front and back wheels on robot in meters
        public static final double WHEEL_BASE = 0.52705;

        // Maximum linear chassis speed in meters per second (MK4 standard modules capable of 4.1)
        public static final double MAX_LINEAR_SPEED = 2;
        public static final double MAX_LINEAR_ACCELERATION = 2;
        // Maximum chassis rotational speed in radians per second
        public static final double rotationRadius = Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * Math.PI;
        public static final double MAX_ROTATION = (MAX_LINEAR_SPEED / rotationRadius) * (2 * Math.PI);
        public static final double MAX_ROT_ACCEL = MAX_ROTATION * 3;

        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2));

        public static final double P_HOLDANGLETELE = 2.5; //.5
        public static final double I_HOLDANGLETELE = 0.25; 
        public static final double D_HOLDANGLETELE = 0;

        public static final double P_HOLDANGLEAUTO = 5; //4
        public static final double I_HOLDANGLEAUTO = .25; //.25
        public static final double D_HOLDANGLEAUTO = 0;

        public static final double P_HOLDTRANSLATION = 1; //1
        public static final double I_HOLDTRANSLATION = 0;
        public static final double D_HOLDTRANSLATION = 0;

        public static final SwerveModuleState STOPPED_STATE = new SwerveModuleState(0, new Rotation2d());

        public static final SwerveModuleState[] X_SHAPE_ARRAY = {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };


        public static final int PigeonID = 01;

        public static final int MODULE_1_DRIVE_ID = 11;
        public static final int MODULE_1_TURN_ID = 12;
        public static final int MODULE_1_ENCODER_ID = 11;
        public static final double MODULE_1_OFFSET = 76.7;

        public static final int MODULE_2_DRIVE_ID = 13;
        public static final int MODULE_2_TURN_ID = 14;
        public static final int MODULE_2_ENCODER_ID = 13;
        public static final double MODULE_2_OFFSET = 156.6;

        public static final int MODULE_3_DRIVE_ID = 15;
        public static final int MODULE_3_TURN_ID = 16;
        public static final int MODULE_3_ENCODER_ID = 15;
        public static final double MODULE_3_OFFSET = 59.8;

        public static final int MODULE_4_DRIVE_ID = 17;
        public static final int MODULE_4_TURN_ID = 18;
        public static final int MODULE_4_ENCODER_ID = 17;
        public static final double MODULE_4_OFFSET = -153.1;
    }

    public static final class ControllerConversions{
        public static final double DEADBAND = 0.025;
        public static final UnaryOperator<Double> conversionFunction = (input) -> (Math.copySign(input * input, input));
    }

    public static final class Controllers {

    }
    
    public static void configureMotor(WPI_TalonFX motor, boolean braked, boolean reversed) {
        motor.configFactoryDefault();
        motor.configSupplyCurrentLimit(CURRENT_LIMIT);
        motor.setInverted(reversed);
        motor.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Default configuration for a motor. Assumes that the motor should be braked, but not reversed
     * @param motor Talon to configure
     */
    public static void configureMotor(WPI_TalonFX motor) {configureMotor(motor, true, false);}
}
