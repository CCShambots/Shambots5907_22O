// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.AllianceColor.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.math.Range;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.math.BoundingRegion;
import frc.robot.util.math.Range;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Map;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import static frc.robot.Constants.SwerveDrivetrain.moduleOffsets;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //TODO: Pick actual CANdle ID

    public static class Lights {
        public static final int CANdleID = 0;
        public static final int numLEDs = 300;
        public static final double blinkingSpeed = 0.5;
        public static final double bounceSpeed = 0.5;
        public static final double fasterBlinkingSpeed = 0.75;
        public static final Animation lightsOff = new FireAnimation();
        public static final Animation idleAnimation = new  LarsonAnimation(0, 0, 255, 0, bounceSpeed, numLEDs, BounceMode.Front, 7);
        public static final Animation testingAnimation = new StrobeAnimation(255, 255, 0, 0, blinkingSpeed, numLEDs);
        public static final Animation oneBallAnimation = new StrobeAnimation(0, 0, 255, 0, blinkingSpeed, numLEDs);
        public static final Animation twoBallAnimation = new StrobeAnimation(0, 0, 255, 0, 0, numLEDs); // Value of zero here might not work
        public static final Animation lockedInOneBall = new StrobeAnimation(0, 255, 0, 0, blinkingSpeed, numLEDs);
        public static final Animation lockedInTwoBall = new StrobeAnimation(0, 255, 0, 0, 0, numLEDs);
        public static final Animation bottomEjectAnimation = new StrobeAnimation(255, 0, 0, 0, fasterBlinkingSpeed, numLEDs);
        public static final Animation climbingAnimation = new StrobeAnimation(255, 0, 255, 0, blinkingSpeed, numLEDs);
    }


    public static AllianceColor alliance = Red;

    public static int COMPRESSOR_ID = 0;

    public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1); //enable these limits, current limit, trigger threshold, trigger threshold time

    public static final class Climber {
        //TODO: Get actual motor id's
        public static final int MOTOR_1_ID = 5;
        public static final int MOTOR_2_ID = 6;
    public static final Translation2d GOAL_POS = new Translation2d(8.23, 4.11); //in meters

    public static final class SwerveModule{
        // Physical Constants
        public static final double WHEEL_RADIUS = 0.0508;
        // Use this ratio to convert from CANCoder absolute position to radians
        public static final double TURN_SENSOR_RATIO = 1;

        // Drive motor PID controller coefficients
        public static final double P_DRIVE = 3;
        public static final double I_DRIVE = 0;
        public static final double D_DRIVE = 0;
        public static final double KS_DRIVE = .75;
        public static final double KV_DRIVE = 2.3;

        // Use this ratio to convert from Falcon angular velocity to wheel angular velocity
        public static final double DRIVE_RATIO = 8.14;

        // Turn motor PID controller coefficients
        // using a trapezoidal profile
        public static final double P_TURN = 4.25;
        public static final double I_TURN = 0;
        public static final double D_TURN = 0.06;
        public static final double KS_TURN = .7;
        public static final double KV_TURN = 0.20;
        public static final double MAX_TURN_SPEED = 50; // Rad/S //50
        public static final double MAX_TURN_ACCEL = 400; // Rad/S^2
    }

    public static final class SwerveDrivetrain{

        // Distance between centers of right and left wheels on robot in meters
        public static final double TRACK_WIDTH = 0.529;
        // Distance between front and back wheels on robot in meters
        public static final double WHEEL_BASE = 0.614;

        // Maximum linear chassis speed in meters per second (MK4 standard modules capable of 4.1)
        public static final double MAX_LINEAR_SPEED = 2;
        public static final double MAX_LINEAR_ACCELERATION = 2;
        // Maximum chassis rotational speed in radians per second
        public static final double rotationRadius = Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * Math.PI;
        public static final double MAX_ROTATION = (MAX_LINEAR_SPEED / rotationRadius) * (2 * Math.PI);
        public static final double MAX_ROT_ACCEL = MAX_ROTATION * 3;

        public static final Translation2d[] moduleOffsets = {
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2)
        };

        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(moduleOffsets);

        public static final double P_HOLDANGLETELE = .5;
        public static final double I_HOLDANGLETELE = 0.; //.25
        public static final double D_HOLDANGLETELE = 0;

        public static final double P_HOLDANGLEAUTO = 4;
        public static final double I_HOLDANGLEAUTO = .25;
        public static final double D_HOLDANGLEAUTO = 0;

        public static final double P_HOLDTRANSLATION = 1;
        public static final double I_HOLDTRANSLATION = 0;
        public static final double D_HOLDTRANSLATION = 0;

        public static final SwerveModuleState STOPPED_STATE = new SwerveModuleState(0, new Rotation2d());

        public static final SwerveModuleState[] X_SHAPE_ARRAY = {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };

        public static Supplier<Pose2d> getOdoPose;
        public static Supplier<Rotation2d> getDrivetrainAngle;
    }

    public static final class ControllerConversions{
        public static final double DEADBAND = 0.025;
        public static final UnaryOperator<Double> conversionFunction = (input) -> (Math.copySign(input * input, input));
    }

    public static final class Turret {
        //TODO: Update CAN ID's and digital pins
        public static final int ROTARY_MOTOR_ID = 21;
        public static final int HOOD_MOTOR_ID = 22;
        public static final int FLYWHEEL_MOTOR1_ID = 23;
        public static final int FLYWHEEL_MOTOR2_ID = 24;

        public static final int LIM_SWITCH_1_ID = 8;
        public static final int LIM_SWITCH_2_ID = 9;

        public static final double FLYWHEEL_KS = 0;
        public static final double FLYWHEEL_KV = 0;
        public static final double FLYWHEEL_KP = 0;
        public static final double FLYWHEEL_KI = 0;
        public static final double FLYWHEEL_KD = 0;

        public static final double FLYWHEEL_TOLERANCE = 50;

        public static final double ROTARY_KS = 0;
        public static final double ROTARY_KV = 0;
        public static final double ROTARY_KP = 0;
        public static final double ROTARY_KI = 0;
        public static final double ROTARY_KD = 0;

        public static final double ROTARY_MAX_VEL = 0; //Deg/sec
        public static final double ROTARY_MAX_ACCEL = 0; //Deg/(sec^2)

        public static final TrapezoidProfile.Constraints NORMAL_ROTARY_CONSTRAINTS = new TrapezoidProfile.Constraints(ROTARY_MAX_VEL, ROTARY_MAX_ACCEL);

        public static final double ROTARY_SEARCH_VEL = 0; //Deg/sec
        public static final double ROTARY_SEARCH_ACCEL = 0; //Deg/(sec^2)

        public static final TrapezoidProfile.Constraints SEARCH_ROTARY_CONSTRAINTS = new TrapezoidProfile.Constraints(ROTARY_SEARCH_VEL, ROTARY_SEARCH_ACCEL);

        public static final double ROTARY_CLOCKWISE_LIMIT = -135;
        public static final double ROTARY_COUNTER_CLOCKWISE_LIMIT = 135;

        public static final Range ROTARY_RANGE = new Range(ROTARY_CLOCKWISE_LIMIT, ROTARY_COUNTER_CLOCKWISE_LIMIT);

        public static final double ROTARY_TOLERANCE = 2; //Degrees
        public static final double LIMELIGHT_DEADBAND = 2; //Degrees

        public static final double HOOD_KS = 0;
        public static final double HOOD_KV = 0;
        public static final double HOOD_KP = 0;
        public static final double HOOD_KI = 0;
        public static final double HOOD_KD = 0;
        public static final double HOOD_MAX_VEL = 0; //Deg/sec
        public static final double HOOD_MAX_ACCEL = 0; //Deg/(sec^2)

        public static final double HOOD_MIN_ANGLE = 0;
        public static final double HOOD_MAX_ANGLE = 32;

        public static final Range HOOD_RANGE = new Range(HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);

        public static final double HOOD_TOLERANCE = 0.5;

        //TODO: Fill in
        public static final double LIMELIGHT_HEIGHT = 2; //Meters
        public static final double LIMELIGHT_ANGLE = Math.toRadians(30); //Radians
        public static final double GOAL_HEIGHT = 5; //Meters (middle of the tape on the upper goal)
        public static final double GOAL_TO_CENTER_DISTANCE = 2; //Meters

        public static final Translation2d OUR_HANGAR_POINT = new Translation2d(1.75, 6.5);

        public static final Translation2d OPP_HANGAR_POINT = new Translation2d(14.71, 1.75);
        public static final BoundingRegion INVALID_EJECTION_REGION = new BoundingRegion(
                new Translation2d(8.75, 2.5),
                new Translation2d(13, 0),
                new Translation2d(16.5,  0),
                new Translation2d(16.5, 5.5),
                new Translation2d(7.75, 5.5)
        );

        public static Supplier<Rotation2d> getLimelightXOffsetAngle;
        public static Supplier<Rotation2d> getRotaryAngle;
    }

        public static final int SOLENOID_1_PORT = 1;
        public static final int SOLENOID_2_PORT = 2;


        //TODO: Maybe no load PID's (hopefully not though)
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final double MAX_VEL = 0; //in/sec
        public static final double MAX_ACCEL = 0; //in/sec^2

        public static final double LOWERED_POS = 0;
        public static final double RIASED_POS = 20; //TODO: Calculate amount of inches

        public static final double MINIMUM_POWER = 0.25; //Power at which the climber will still move
    }
    public static final class Drivetrain {

    }

    public static final class Intake {
        public static final int LEFT_MOTOR_ID = 11;
        public static final int RIGHT_MOTOR_ID = 12;
        public static final int LEFT_SOLENOID_FORWARD = 0;
        public static final int LEFT_SOLENOID_REVERSE = 1;
        public static final int RIGHT_SOLENOID_FORWARD = 2;
        public static final int RIGHT_SOLENOID_REVERSE = 3;

        //TODO: Increase speed
        public static final double INTAKE_POWER = 0.25;

        public static final double MANUAL_EJECT_TIME = 3; //Seconds

    }

    public static final class Conveyor {
        public static final int LEFT_COMPACTOR_ID = 21;
        public static final int RIGHT_COMPACTOR_ID = 22;
        public static final int LEFT_CONVEYOR_ID = 23;
        public static final int RIGHT_CONVEYOR_ID = 24;
        public static final int LEFT_COLOR_ONE = 0;
        public static final int LEFT_COLOR_TWO = 1;
        public static final int RIGHT_COLOR_ONE = 2;
        public static final int RIGHT_COLOR_TWO = 3;
        public static final int LEFT_PROX = 4;
        public static final int RIGHT_PROX = 5;
        public static final int CENTER_PROX = 6;

        //TODO: Raise both of these speeds
        public static final double CONVEYOR_SPEED = 0.25;
        public static final double COMPACTOR_SPEED = 0.25;
    }

    public static final Translation2d GOAL_POS = new Translation2d(8.23, 4.11); //in meters

    public static final class SwerveModule{
        // Physical Constants
        public static final double WHEEL_RADIUS = 0.0508;
        // Use this ratio to convert from CANCoder absolute position to radians
        public static final double TURN_SENSOR_RATIO = 1;

        // Drive motor PID controller coefficients
        public static final double P_DRIVE = 3;
        public static final double I_DRIVE = 0;
        public static final double D_DRIVE = 0;
        public static final double KS_DRIVE = .75;
        public static final double KV_DRIVE = 2.3;

        // Use this ratio to convert from Falcon angular velocity to wheel angular velocity
        public static final double DRIVE_RATIO = 8.14;

        // Turn motor PID controller coefficients
        // using a trapezoidal profile
        public static final double P_TURN = 4.25;
        public static final double I_TURN = 0;
        public static final double D_TURN = 0.06;
        public static final double KS_TURN = .7;
        public static final double KV_TURN = 0.20;
        public static final double MAX_TURN_SPEED = 50; // Rad/S //50
        public static final double MAX_TURN_ACCEL = 400; // Rad/S^2
    }

    public static final class SwerveDrivetrain{

        // Distance between centers of right and left wheels on robot in meters
        public static final double TRACK_WIDTH = 0.529;
        // Distance between front and back wheels on robot in meters
        public static final double WHEEL_BASE = 0.614;

        // Maximum linear chassis speed in meters per second (MK4 standard modules capable of 4.1)
        public static final double MAX_LINEAR_SPEED = 2;
        public static final double MAX_LINEAR_ACCELERATION = 2;
        // Maximum chassis rotational speed in radians per second
        public static final double rotationRadius = Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * Math.PI;
        public static final double MAX_ROTATION = (MAX_LINEAR_SPEED / rotationRadius) * (2 * Math.PI);
        public static final double MAX_ROT_ACCEL = MAX_ROTATION * 3;

        public static final Translation2d[] moduleOffsets = {
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2)
        };

        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(moduleOffsets);

        public static final double P_HOLDANGLETELE = .5;
        public static final double I_HOLDANGLETELE = 0.; //.25
        public static final double D_HOLDANGLETELE = 0;

        public static final double P_HOLDANGLEAUTO = 4;
        public static final double I_HOLDANGLEAUTO = .25;
        public static final double D_HOLDANGLEAUTO = 0;

        public static final double P_HOLDTRANSLATION = 1;
        public static final double I_HOLDTRANSLATION = 0;
        public static final double D_HOLDTRANSLATION = 0;

        public static final SwerveModuleState STOPPED_STATE = new SwerveModuleState(0, new Rotation2d());

        public static final SwerveModuleState[] X_SHAPE_ARRAY = {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };

        public static Supplier<Pose2d> getOdoPose;
        public static Supplier<Rotation2d> getDrivetrainAngle;
    }

    public static final class ControllerConversions{
        public static final double DEADBAND = 0.025;
        public static final UnaryOperator<Double> conversionFunction = (input) -> (Math.copySign(input * input, input));
    }

    public static final class Turret {
        //TODO: Update CAN ID's and digital pins
        public static final int ROTARY_MOTOR_ID = 21;
        public static final int HOOD_MOTOR_ID = 22;
        public static final int FLYWHEEL_MOTOR1_ID = 23;
        public static final int FLYWHEEL_MOTOR2_ID = 24;

        public static final int LIM_SWITCH_1_ID = 8;
        public static final int LIM_SWITCH_2_ID = 9;

        public static final double FLYWHEEL_KS = 0;
        public static final double FLYWHEEL_KV = 0;
        public static final double FLYWHEEL_KP = 0;
        public static final double FLYWHEEL_KI = 0;
        public static final double FLYWHEEL_KD = 0;

        public static final double FLYWHEEL_TOLERANCE = 50;

        public static final double ROTARY_KS = 0;
        public static final double ROTARY_KV = 0;
        public static final double ROTARY_KP = 0;
        public static final double ROTARY_KI = 0;
        public static final double ROTARY_KD = 0;

        public static final double ROTARY_MAX_VEL = 0; //Deg/sec
        public static final double ROTARY_MAX_ACCEL = 0; //Deg/(sec^2)

        public static final TrapezoidProfile.Constraints NORMAL_ROTARY_CONSTRAINTS = new TrapezoidProfile.Constraints(ROTARY_MAX_VEL, ROTARY_MAX_ACCEL);

        public static final double ROTARY_SEARCH_VEL = 0; //Deg/sec
        public static final double ROTARY_SEARCH_ACCEL = 0; //Deg/(sec^2)

        public static final TrapezoidProfile.Constraints SEARCH_ROTARY_CONSTRAINTS = new TrapezoidProfile.Constraints(ROTARY_SEARCH_VEL, ROTARY_SEARCH_ACCEL);

        public static final double ROTARY_CLOCKWISE_LIMIT = -135;
        public static final double ROTARY_COUNTER_CLOCKWISE_LIMIT = 135;

        public static final Range ROTARY_RANGE = new Range(ROTARY_CLOCKWISE_LIMIT, ROTARY_COUNTER_CLOCKWISE_LIMIT);

        public static final double ROTARY_TOLERANCE = 2; //Degrees
        public static final double LIMELIGHT_DEADBAND = 2; //Degrees

        public static final double HOOD_KS = 0;
        public static final double HOOD_KV = 0;
        public static final double HOOD_KP = 0;
        public static final double HOOD_KI = 0;
        public static final double HOOD_KD = 0;
        public static final double HOOD_MAX_VEL = 0; //Deg/sec
        public static final double HOOD_MAX_ACCEL = 0; //Deg/(sec^2)

        public static final double HOOD_MIN_ANGLE = 0;
        public static final double HOOD_MAX_ANGLE = 32;

        public static final Range HOOD_RANGE = new Range(HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);

        public static final double HOOD_TOLERANCE = 0.5;

        //TODO: Fill in
        public static final double LIMELIGHT_HEIGHT = 2; //Meters
        public static final double LIMELIGHT_ANGLE = Math.toRadians(30); //Radians
        public static final double GOAL_HEIGHT = 5; //Meters (middle of the tape on the upper goal)
        public static final double GOAL_TO_CENTER_DISTANCE = 2; //Meters

        public static Supplier<Rotation2d> getLimelightXOffsetAngle;
        public static Supplier<Rotation2d> getRotaryAngle;
    }

    public static void configureMotor(WPI_TalonFX motor, boolean braked, boolean reversed, boolean configFactoryDefault) {
        if(configFactoryDefault) motor.configFactoryDefault();
        motor.configSupplyCurrentLimit(CURRENT_LIMIT);
        motor.setInverted(reversed);
        motor.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Default configuration for a motor. Assumes that the motor should be braked, but not reversed
     * @param motor Talon to configure
     */
    public static void configureMotor(WPI_TalonFX motor) {configureMotor(motor, true, false, true);}

    /**
     * Default configuration for a motor. Assumes that the motor should be braked, but not reversed
     * @param motor Talon to configure
     */
    public static void configureMotor(WPI_TalonFX motor, boolean configFactoryDefault) {configureMotor(motor, true, false, configFactoryDefault);}

    public static void pullAllianceFromFMS() {
        boolean isRedAlliance = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
        alliance = isRedAlliance ? Red: Blue;
    }

        public enum AllianceColor {
            Red, Blue
        }


    }
