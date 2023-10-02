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
        public static final double SPIT_SETPOINT = 10.0;
        public static final double INTAKE_SETPOINT = SPIT_SETPOINT;
        public static final double CUBE_MID = 75;

        public static final double SCOREHIGH_SETPOINT = 104;
        public static final double SCOREMID_SETPOINT = 75;

        public static final double OUTPUT_LIMIT = 0.4;
        public static final double kP = 0.011; // 0.04 / 4;
        public static final double kD = 0.9;
        public static final double kG = 0.25;

        public static final double DEGREE_RANGE = 111.0; // the overall range of the pivot from cad
        public static final double TICK_RANGE = 16.545471; // the corresponding range of ticks that the pivot can move
        public static final double DEGREES_PER_TICK = DEGREE_RANGE / TICK_RANGE;
        public static final double GOHIGH_SETPOINT = 104;
        public static final double GOMID_SETPOINT = 75;
    }

    // -----------------------------------------------------------------------\\

    public static final class ArmConstants {
        public static final int ARM_MOTOR = 2;

        public static final double ARM_RETRACTED = 4;
        public static final double ARM_EXTENDED = 0;
        public static final double ARM_MID = 26.4;
        public static final double ARM_HIGH = 20.3;
        public static final double ARM_SPIT = 36.3;
        public static final double ARM_CALIBRATE = 0;
        public static final double kP = 0.08;
        public static final double OUTPUT_LIMIT = 0.4;

    }
    // ---------------------------------------------------------------\\

    public class IntakeConstants {
        public static final int INTAKE_ROLLLERS = 1;
        public static final double INTAKE = -0.5;
        public static final double SPIT = 0.5;
        public static final double IDLE = 0;
    }
}
