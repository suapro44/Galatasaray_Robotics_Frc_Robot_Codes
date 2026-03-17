package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

/**
 * Swerve drivetrain subsystem using 4 MK5n modules driven by
 * CTRE TalonFX (Kraken X60) motors, CANcoders, and a Pigeon 2.0 IMU.
 *
 * Provides field-centric drive capabilities.
 */
public class DrivetrainSubsystem extends SubsystemBase {

    // ── Swerve Modules ──
    private final SwerveModule[] modules;

    // ── IMU ──
    private final Pigeon2 pigeon;

    // ── Kinematics & Odometry ──
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    public DrivetrainSubsystem() {
        // Pigeon 2.0
        pigeon = new Pigeon2(DriveConstants.PIGEON_ID);
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.setYaw(0.0);

        // Create 4 swerve modules (FL, FR, BL, BR)
        modules = new SwerveModule[] {
            new SwerveModule(
                "FL",
                DriveConstants.FL_DRIVE_ID,
                DriveConstants.FL_STEER_ID,
                DriveConstants.FL_ENCODER_ID,
                DriveConstants.FL_ENCODER_OFFSET
            ),
            new SwerveModule(
                "FR",
                DriveConstants.FR_DRIVE_ID,
                DriveConstants.FR_STEER_ID,
                DriveConstants.FR_ENCODER_ID,
                DriveConstants.FR_ENCODER_OFFSET
            ),
            new SwerveModule(
                "BL",
                DriveConstants.BL_DRIVE_ID,
                DriveConstants.BL_STEER_ID,
                DriveConstants.BL_ENCODER_ID,
                DriveConstants.BL_ENCODER_OFFSET
            ),
            new SwerveModule(
                "BR",
                DriveConstants.BR_DRIVE_ID,
                DriveConstants.BR_STEER_ID,
                DriveConstants.BR_ENCODER_ID,
                DriveConstants.BR_ENCODER_OFFSET
            )
        };

        kinematics = new SwerveDriveKinematics(
            DriveConstants.FL_POSITION,
            DriveConstants.FR_POSITION,
            DriveConstants.BL_POSITION,
            DriveConstants.BR_POSITION
        );

        odometry = new SwerveDriveOdometry(
            kinematics,
            getGyroRotation2d(),
            getModulePositions()
        );
    }

    // ────────────── PUBLIC API ──────────────

    /**
     * Drive the robot using field-centric control.
     *
     * @param xSpeed        Forward speed (m/s, positive = away from alliance wall).
     * @param ySpeed        Strafe speed (m/s, positive = left).
     * @param rotationSpeed Rotation rate (rad/s, positive = counter-clockwise).
     * @param fieldRelative Whether speeds are field-relative.
     */
    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getGyroRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_SPEED);

        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    /** Stop all modules immediately. */
    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    /** Reset the gyroscope heading to zero (call when facing downfield). */
    public void zeroHeading() {
        pigeon.setYaw(0.0);
    }

    /** Reset odometry to a given pose. */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
    }

    /** @return The current estimated pose from odometry. */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /** @return The gyro heading as a {@link Rotation2d}. */
    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
    }

    /** @return Current heading in degrees (0–360). */
    public double getHeading() {
        return getGyroRotation2d().getDegrees();
    }

    // ────────────── PERIODIC ──────────────

    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getModulePositions());

        SmartDashboard.putNumber("Drivetrain/Heading", getHeading());
        SmartDashboard.putString("Drivetrain/Pose", getPose().toString());
    }

    // ────────────── HELPERS ──────────────

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    // ══════════════════════════════════════════
    //  INNER CLASS — Single Swerve Module
    // ══════════════════════════════════════════
    private static class SwerveModule {

        private final String name;
        private final TalonFX driveMotor;
        private final TalonFX steerMotor;
        private final CANcoder absoluteEncoder;
        private final double encoderOffset;

        private final VelocityVoltage driveRequest  = new VelocityVoltage(0);
        private final PositionVoltage steerRequest   = new PositionVoltage(0);

        SwerveModule(String name, int driveId, int steerId, int encoderId, double offset) {
            this.name = name;
            this.encoderOffset = offset;

            // Drive motor config
            driveMotor = new TalonFX(driveId);
            TalonFXConfiguration driveConfig = new TalonFXConfiguration();
            driveConfig.Slot0.kP = DriveConstants.DRIVE_KP;
            driveConfig.Slot0.kI = DriveConstants.DRIVE_KI;
            driveConfig.Slot0.kD = DriveConstants.DRIVE_KD;
            driveConfig.Slot0.kV = DriveConstants.DRIVE_KV;
            driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            driveMotor.getConfigurator().apply(driveConfig);

            // Steer motor config
            steerMotor = new TalonFX(steerId);
            TalonFXConfiguration steerConfig = new TalonFXConfiguration();
            steerConfig.Slot0.kP = DriveConstants.STEER_KP;
            steerConfig.Slot0.kI = DriveConstants.STEER_KI;
            steerConfig.Slot0.kD = DriveConstants.STEER_KD;
            steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
            steerMotor.getConfigurator().apply(steerConfig);

            // CANcoder
            absoluteEncoder = new CANcoder(encoderId);
            CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
            absoluteEncoder.getConfigurator().apply(encoderConfig);
        }

        /**
         * Set the desired state for this module.
         * Optimises the target to minimise turning (via {@link SwerveModuleState#optimize}).
         */
        void setDesiredState(SwerveModuleState desiredState) {
            // Get current steer angle from CANcoder
            double currentAngle = getSteerAngle();
            Rotation2d currentRotation = Rotation2d.fromRotations(currentAngle);

            // Optimise — may reverse drive direction to avoid > 90° turns
            desiredState.optimize(currentRotation);

            // Drive: convert m/s → rotor rotations/s
            double driveRPS = desiredState.speedMetersPerSecond
                / DriveConstants.WHEEL_CIRCUMFERENCE
                * DriveConstants.DRIVE_GEAR_RATIO;
            driveMotor.setControl(driveRequest.withVelocity(driveRPS));

            // Steer: target position in rotations
            double targetRotations = desiredState.angle.getRotations();
            steerMotor.setControl(steerRequest.withPosition(targetRotations));
        }

        void stop() {
            driveMotor.stopMotor();
            steerMotor.stopMotor();
        }

        /** @return Current steer angle in rotations (from CANcoder). */
        double getSteerAngle() {
            return absoluteEncoder.getAbsolutePosition().getValueAsDouble() - encoderOffset;
        }

        /** @return Drive distance in metres and steer angle. */
        SwerveModulePosition getPosition() {
            double driveRotations = driveMotor.getPosition().getValueAsDouble();
            double distanceMetres = (driveRotations / DriveConstants.DRIVE_GEAR_RATIO)
                * DriveConstants.WHEEL_CIRCUMFERENCE;
            return new SwerveModulePosition(distanceMetres, Rotation2d.fromRotations(getSteerAngle()));
        }
    }
}
