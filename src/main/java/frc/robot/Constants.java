// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.math.BoundingRegion;
import frc.robot.ShamLib.motors.PIDFGains;
import frc.robot.util.math.Range;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.function.IntSupplier;
import java.util.function.Supplier;
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

    public static AllianceColor alliance = AllianceColor.Red;
    public static RobotEnabled botEnabledStatus = RobotEnabled.Disabled;

    public static int COMPRESSOR_ID = 01;

    public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1); //enable these limits, current limit, trigger threshold, trigger threshold time
    public static final Translation2d GOAL_POS = new Translation2d(8.23, 4.11); //in meters

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

    public static final class Conveyor {
        public static final int LEFT_COMPACTOR_ID = 31;
        public static final int RIGHT_COMPACTOR_ID = 32;
        public static final int LEFT_CONVEYOR_ID = 33;
        public static final int RIGHT_CONVEYOR_ID = 34;
        public static final int LEFT_COLOR_ONE = 4;
        public static final int LEFT_COLOR_TWO = 5;
        public static final int RIGHT_COLOR_ONE = 6;
        public static final int RIGHT_COLOR_TWO = 7;
        public static final int LEFT_PROX = 2;
        public static final int RIGHT_PROX = 0;
        public static final int CENTER_PROX = 1;

        public static final double CONVEYOR_SPEED = 0.45; //0.65
        public static final double COMPACTOR_SPEED = 0.5; //0.5

        public static IntSupplier numBallsSupplier;
        public static Supplier<frc.robot.subsystems.Conveyor> conveyorSupplier;
    }

    public static final class Climber {
        public static final int MOTOR_1_ID = 51;
        public static final int MOTOR_2_ID = 52;

        public static final int SOLENOID_1_PORT = 2;

        public static final PIDFGains climberGains = new PIDFGains(0.1, 0.0, 0.0, 0.0465, 0, 1.0);

        public static final double climberRatio =
                (1.0 / 2048.0) * //Motor revolutions
                (12.0 / 50.0) * //Stage 1
                (14.0 / 60.0) * //Stage 2
                1.751 * Math.PI //Pitch diameter (1.751") --> distance traveled by chain
        ;

        public static final double MAX_VEL = 2; //in/sec
        public static final double MAX_ACCEL = 2; //in/sec^2

        public static final double LOWERED_POS = 0;
        public static final double RIASED_POS = 18.25; 

        public static final double MINIMUM_POWER = 0.25; //Power at which the climber will still move

        public static final double TOLERANCE = 0.25; //Inches
    }

    public static final class SwerveModule{
        // Physical Constants
        public static final double WHEEL_RADIUS = 0.0508;
        // Use this ratio to convert from CANCoder absolute position to radians
        public static final double TURN_SENSOR_RATIO = 1.0/(150.0/7.0);

        // Drive motor PID controller coefficients
        public static final double P_DRIVE = 0.175;
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

        public static final PIDFGains turnGains = new PIDFGains(
                P_TURN,
                I_TURN,
                D_TURN,
                KV_TURN,
                0, //TODO: change this if needed
                20 //TODO: change this
        );

        public static final PIDFGains driveGains = new PIDFGains(
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
        public static final double MAX_LINEAR_ACCELERATION = 5;
        // Maximum chassis rotational speed in radians per second
        public static final double rotationRadius = Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * Math.PI;
        public static final double MAX_ROTATION = (MAX_LINEAR_SPEED / rotationRadius) * (2 * Math.PI);
        public static final double MAX_ROT_ACCEL = MAX_ROTATION * 3;
        
        public static final Translation2d[] moduleOffsets = {
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)
        };

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(moduleOffsets);

        public static final double P_HOLDANGLETELE = 3.5; //.5
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

        public static Supplier<Pose2d> getOdoPose;
        public static Supplier<Rotation2d> getDrivetrainAngle;
    }

    public static final class ControllerConversions{
        public static final double DEADBAND = 0.025;
        public static final UnaryOperator<Double> conversionFunction = (input) -> (Math.copySign(input * input, input));
    }

    public static final class Turret {
        public static final int ROTARY_MOTOR_ID = 41;
        public static final int HOOD_MOTOR_ID = 42;
        public static final int FLYWHEEL_MOTOR1_ID = 43;
        public static final int FLYWHEEL_MOTOR2_ID = 44;

        public static final int OUTER_LIM_SWITCH_ID = 8;
        public static final int CENTER_LIM_SWITCH_ID = 9;

        public static final double MAX_SHOOT_TIME = 1.3; //seconds

        public static final int flywheelkSlotIdx = 0;

        /**
         * Talon FX supports multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int flywheelkPIDLoopIdx = 0;

        /**
         * set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int flywheelkTimeoutMs = 30;

        /**
         * Gains used in Motion Magic, to be adjusted accordingly
         * Gains(kp, ki, kd, kf, izone, peak output);
         */
        public static final PIDFGains flywheelGains = new PIDFGains(0.5254, 0.0, 0.275, 0.0575, 0, 1.0);
        public static final double FLYWHEEL_TOLERANCE = 30;

        public static final double SHOOT_DELAY = 1;


	    public static final int rotarykSlotIdx = 0;

        /**
         * Talon FX supports multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int rotarykPIDLoopIdx = 0;

        /**
         * set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int rotarykTimeoutMs = 30;

        /**
         * Gains used in Motion Magic, to be adjusted accordingly
         * Gains(kp, ki, kd, kf, izone, peak output);
         */
        public static final PIDFGains rotaryGains = new PIDFGains(0.15, 0.0, 0.0, 0.45, 0, 1.0);

        public static final double ROTARY_MAX_VEL = 90; //Deg/sec
        public static final double ROTARY_MAX_ACCEL = 180; //Deg/(sec^2)

        public static final TrapezoidProfile.Constraints NORMAL_ROTARY_CONSTRAINTS = new TrapezoidProfile.Constraints(ROTARY_MAX_VEL, ROTARY_MAX_ACCEL);

        public static final double ROTARY_SEARCH_VEL = 0; //Deg/sec
        public static final double ROTARY_SEARCH_ACCEL = 0; //Deg/(sec^2)

        public static final TrapezoidProfile.Constraints SEARCH_ROTARY_CONSTRAINTS = new TrapezoidProfile.Constraints(ROTARY_SEARCH_VEL, ROTARY_SEARCH_ACCEL);

        public static final double ROTARY_CLOCKWISE_LIMIT = -135;
        public static final double ROTARY_COUNTER_CLOCKWISE_LIMIT = 90;

        public static final Range ROTARY_RANGE = new Range(ROTARY_CLOCKWISE_LIMIT, ROTARY_COUNTER_CLOCKWISE_LIMIT);

        public static final double ROTARY_TOLERANCE = 2; //Degrees
        public static final double LIMELIGHT_DEADBAND = 2; //Degrees

        public static final PIDFGains hoodGains = new PIDFGains(0.065, 0, 0, 0.075, 0, 1.0);

        public static final double HOOD_KS = 0.75;
        public static final double HOOD_KV = 0.0625;
        public static final double HOOD_KP = 0.125;
        public static final double HOOD_KI = 0;
        public static final double HOOD_KD = 0;
        public static final double HOOD_MAX_VEL = 64; //Deg/sec
        public static final double HOOD_MAX_ACCEL = 64; //Deg/(sec^2)

        public static final double HOOD_MIN_ANGLE = 0;
        public static final double HOOD_MAX_ANGLE = 30;

        public static final Range HOOD_RANGE = new Range(HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);

        public static final double HOOD_TOLERANCE = 0.5;
        public static final double LIMELIGHT_TOLERANCE = 4;

        public static final double LIMELIGHT_HEIGHT = 0.81; //Meters
        public static final double LIMELIGHT_ANGLE = Math.toRadians(30); //Radians
        public static final double GOAL_HEIGHT = 2.39; //Meters (middle of the tape on the upper goal)
        public static final double GOAL_TO_CENTER_DISTANCE = 0.61; //Meters

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

    public static final class Intake {
        public static final int LEFT_MOTOR_ID = 21;
        public static final int RIGHT_MOTOR_ID = 22;
        public static final int LEFT_SOLENOID_FORWARD = 3;
        public static final int LEFT_SOLENOID_REVERSE = 4;
        public static final int RIGHT_SOLENOID_FORWARD = 0;
        public static final int RIGHT_SOLENOID_REVERSE = 1;

        public static final double INTAKE_POWER = .65;
        public static final double PUMP_TIME_SECONDS = 0.5;

        public static final double MANUAL_EJECT_TIME = 1; //Seconds
    }

    public static void defaultOptimizeMotor(WPI_TalonFX motor) {
        motor.setStatusFramePeriod(2, 1000);
        motor.setStatusFramePeriod(3, 1000);
        motor.setStatusFramePeriod(4, 1000);
        motor.setStatusFramePeriod(10, 1000);
        motor.setStatusFramePeriod(13, 1000);
        motor.setStatusFramePeriod(14, 1000);
        motor.setStatusFramePeriod(21, 1000);
    }

    public static void optimizeFollowerMotor(WPI_TalonFX motor) {
        defaultOptimizeMotor(motor);
        motor.setStatusFramePeriod(1, 100); //applied motor output
    }

    public static void pullAllianceFromFMS() {
        boolean isRedAlliance = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
        alliance = isRedAlliance ? AllianceColor.Red : AllianceColor.Blue;
    }

    public enum AllianceColor {
        Red, Blue
    }

    public enum RobotEnabled {
        Disabled,
        Teleop,
        Auto,
        Test
    }

}
