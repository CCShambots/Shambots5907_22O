// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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

    public static final int COMPRESSOR_ID = 0;

    public static final class Climber {
        //TODO: Get actual motor id's
        public static final int MOTOR_1_ID = 81;
        public static final int MOTOR_2_ID = 82;

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
