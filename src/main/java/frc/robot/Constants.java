// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CubeShooterConstants {
        public static final int PIVOT_MOTOR = 17;
        public static final int TOP_ROLLER_MOTOR = 15;
        public static final int BOTTOM_ROLLER_MOTOR = 16;

        public static final double RETRACTED_SETPOINT = 104;
        public static final double SPIT_SETPOINT = 5.0;
        public static final double INTAKE_SETPOINT = SPIT_SETPOINT;
        public static final double CUBE_MID = 75;

        public static final double SCOREHIGH_SETPOINT = 1;
        public static final double SCOREMID_SETPOINT = 1;

    }
}
