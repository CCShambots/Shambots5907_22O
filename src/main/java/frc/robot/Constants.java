// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.math.Range;

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

    public static final Translation2d GOAL_POS = new Translation2d(8.23, 4.11); //in meters


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
    }

    public static final class Controllers {

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

}
