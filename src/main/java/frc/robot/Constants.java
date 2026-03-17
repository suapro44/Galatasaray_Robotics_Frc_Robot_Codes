package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Central configuration file holding all robot constants.
 * Organized by subsystem for clarity.
 */
public final class Constants {

    private Constants() {
        // Utility class — prevent instantiation
    }

    // ──────────────────────────────────────────────
    //  DRIVETRAIN (Swerve — MK5n, CTRE Phoenix 6)
    // ──────────────────────────────────────────────
    public static final class DriveConstants {

        // Pigeon 2.0 IMU
        public static final int PIGEON_ID = 0;

        // ── Front-Left Module ──
        public static final int FL_DRIVE_ID  = 1;
        public static final int FL_STEER_ID  = 2;
        public static final int FL_ENCODER_ID = 3;
        public static final double FL_ENCODER_OFFSET = 0.0; // rotations

        // ── Front-Right Module ──
        public static final int FR_DRIVE_ID  = 4;
        public static final int FR_STEER_ID  = 5;
        public static final int FR_ENCODER_ID = 6;
        public static final double FR_ENCODER_OFFSET = 0.0;

        // ── Back-Left Module ──
        public static final int BL_DRIVE_ID  = 7;
        public static final int BL_STEER_ID  = 8;
        public static final int BL_ENCODER_ID = 9;
        public static final double BL_ENCODER_OFFSET = 0.0;

        // ── Back-Right Module ──
        public static final int BR_DRIVE_ID  = 10;
        public static final int BR_STEER_ID  = 11;
        public static final int BR_ENCODER_ID = 12;
        public static final double BR_ENCODER_OFFSET = 0.0;

        // Chassis dimensions (MK5n ~0.57 m trackwidth typical)
        public static final double TRACK_WIDTH  = Units.inchesToMeters(22.5);  // metres
        public static final double WHEEL_BASE   = Units.inchesToMeters(22.5);  // metres

        // Module positions relative to robot centre
        public static final Translation2d FL_POSITION = new Translation2d( WHEEL_BASE / 2.0,  TRACK_WIDTH / 2.0);
        public static final Translation2d FR_POSITION = new Translation2d( WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
        public static final Translation2d BL_POSITION = new Translation2d(-WHEEL_BASE / 2.0,  TRACK_WIDTH / 2.0);
        public static final Translation2d BR_POSITION = new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);

        // MK5n gear ratio & wheel diameter
        public static final double DRIVE_GEAR_RATIO = 5.357; // L3 configuration
        public static final double WHEEL_DIAMETER   = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        // Performance limits
        public static final double MAX_SPEED        = 5.4;   // m/s (theoretical for L3)
        public static final double MAX_ANGULAR_RATE  = 2 * Math.PI; // rad/s

        // Drive motor PID (velocity)
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KV = 0.12;

        // Steer motor PID (position)
        public static final double STEER_KP = 50.0;
        public static final double STEER_KI = 0.0;
        public static final double STEER_KD = 0.5;
    }

    // ──────────────────────────────────
    //  INTAKE (2× NEO — CANSparkMax)
    // ──────────────────────────────────
    public static final class IntakeConstants {
        public static final int LEADER_ID   = 13;
        public static final int FOLLOWER_ID = 14;

        public static final double INTAKE_SPEED = 0.80; // percent output (0–1)

        public static final int CURRENT_LIMIT = 40; // amps
    }

    // ──────────────────────────────────
    //  CHAMBER (1× NEO — CANSparkMax)
    // ──────────────────────────────────
    public static final class ChamberConstants {
        public static final int MOTOR_ID = 15;

        public static final double CHAMBER_SPEED = 0.60; // percent output

        public static final int CURRENT_LIMIT = 30; // amps
    }

    // ──────────────────────────────────────────────────
    //  SHOOTER (3× Vortex wheels + 2× NEO 550 angle)
    // ──────────────────────────────────────────────────
    public static final class ShooterConstants {

        // Shooter wheel motors (CANSparkFlex — Vortex)
        public static final int WHEEL_LEADER_ID    = 16;
        public static final int WHEEL_FOLLOWER1_ID = 17;
        public static final int WHEEL_FOLLOWER2_ID = 18;

        public static final double DEFAULT_SHOOTER_SPEED = 1.0;  // percent output
        public static final int WHEEL_CURRENT_LIMIT = 60; // amps

        // Angle adjustment motors (CANSparkMax — NEO 550)
        public static final int ANGLE_LEADER_ID   = 19;
        public static final int ANGLE_FOLLOWER_ID = 20;

        public static final int ANGLE_CURRENT_LIMIT = 20; // amps

        // Angle PID (SparkPIDController — position control)
        public static final double ANGLE_KP = 0.05;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.002;
        public static final double ANGLE_MIN_OUTPUT = -0.5;
        public static final double ANGLE_MAX_OUTPUT =  0.5;

        // Angle presets (encoder rotations from zero)
        public static final double ANGLE_STOW      = 0.0;
        public static final double ANGLE_SUBWOOFER  = 10.0;
        public static final double ANGLE_PODIUM     = 22.0;
        public static final double ANGLE_AMP        = 35.0;

        // Tolerance for "at setpoint" check
        public static final double ANGLE_TOLERANCE = 0.5; // rotations
    }

    // ────────────────────────────────────
    //  CLIMB (1× NEO — CANSparkMax)
    // ────────────────────────────────────
    public static final class ClimbConstants {
        public static final int MOTOR_ID = 21;

        // PID (SparkPIDController — position control)
        public static final double CLIMB_KP = 0.1;
        public static final double CLIMB_KI = 0.0;
        public static final double CLIMB_KD = 0.005;
        public static final double CLIMB_MIN_OUTPUT = -1.0;
        public static final double CLIMB_MAX_OUTPUT =  1.0;

        // Setpoints (encoder rotations)
        public static final double LEVEL_3_SETPOINT = 150.0;
        public static final double STOW_SETPOINT    = 0.0;

        // Tolerance
        public static final double POSITION_TOLERANCE = 1.0; // rotations

        public static final int CURRENT_LIMIT = 60; // amps
    }

    // ──────────────────────────────────
    //  VISION (2× Limelight)
    // ──────────────────────────────────
    public static final class VisionConstants {
        // NetworkTables table names published by the Limelights
        public static final String MAIN_LIMELIGHT  = "limelight-main";
        public static final String CLIMB_LIMELIGHT = "limelight-climb";
    }

    // ──────────────────────────────────
    //  OPERATOR INTERFACE
    // ──────────────────────────────────
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT   = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double DEADBAND = 0.08;
    }
}
