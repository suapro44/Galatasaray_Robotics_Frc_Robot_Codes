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
 * Swerve Şasi alt sistemi — 4 adet MK5n modül ile çalışır.
 * Motorlar CTRE TalonFX (Kraken X60), Enkoderler CANcoder ve IMU Pigeon 2.0 kullanılarak yazılmıştır.
 *
 * Saha merkezli (field-centric) sürüş kabiliyeti sağlar.
 */
public class DrivetrainSubsystem extends SubsystemBase {

    // ── Swerve Modülleri ──
    private final SwerveModule[] modules;

    // ── IMU ──
    private final Pigeon2 pigeon;

    // ── Kinematik & Odometri (Konum Takibi) ──
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    public DrivetrainSubsystem() {
        // Pigeon 2.0 Jiroskop Kurulumu
        pigeon = new Pigeon2(DriveConstants.PIGEON_ID);
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.setYaw(0.0);

        // 4 adet Swerve Modülünün (Ön Sol, Ön Sağ, Arka Sol, Arka Sağ) oluşturulması
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

    // ────────────── GENEL FONKSİYONLAR (PUBLIC API) ──────────────

    /**
     * Saha merkezli (veya robot merkezli) şasi kontrolü sağlar.
     *
     * @param xSpeed        İleri/Geri hızı (m/s, pozitif değer ittifak duvarından uzaklaşmayı temsil eder).
     * @param ySpeed        Sağa/Sola kayma (strafe) hızı (m/s, pozitif değer sola gidişi temsil eder).
     * @param rotationSpeed Dönme (rotasyon) hızı (rad/s, pozitif değer saat yönünün tersini temsil eder).
     * @param fieldRelative True ise saha merkezli, false ise robot merkezli sürüş.
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

    /** Tüm modülleri anında durdurur. */
    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    /** Jiroskop yönünü sıfırlar (Robot karşı tarafa düz bakarken çağrılmalıdır). */
    public void zeroHeading() {
        pigeon.setYaw(0.0);
    }

    /** Robotun odometri (konum) verisini belirli bir Pose (konum+yön) değerine eşitler. */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
    }

    /** @return Odometriden hesaplanan robotun sahadaki tahmini konumu. */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /** @return Rotation2d formatında robotun mevcut gyro açısı. */
    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
    }

    /** @return Derece cinsinden mevcut açı yönü (0-360). */
    public double getHeading() {
        return getGyroRotation2d().getDegrees();
    }

    // ────────────── PERİYODİK (PERIODIC) ──────────────

    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getModulePositions());

        SmartDashboard.putNumber("Drivetrain/Heading", getHeading());
        SmartDashboard.putString("Drivetrain/Pose", getPose().toString());
    }

    // ────────────── YARDIMCI FONKSİYONLAR (HELPERS) ──────────────

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    // ══════════════════════════════════════════
    //  İÇ SINIF (INNER CLASS) — Tekil Swerve Modülü
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

            // Sürüş (Drive) Motoru Ayarları
            driveMotor = new TalonFX(driveId);
            TalonFXConfiguration driveConfig = new TalonFXConfiguration();
            driveConfig.Slot0.kP = DriveConstants.DRIVE_KP;
            driveConfig.Slot0.kI = DriveConstants.DRIVE_KI;
            driveConfig.Slot0.kD = DriveConstants.DRIVE_KD;
            driveConfig.Slot0.kV = DriveConstants.DRIVE_KV;
            driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            driveMotor.getConfigurator().apply(driveConfig);

            // Yönlendirme (Steer) Motoru Ayarları
            steerMotor = new TalonFX(steerId);
            TalonFXConfiguration steerConfig = new TalonFXConfiguration();
            steerConfig.Slot0.kP = DriveConstants.STEER_KP;
            steerConfig.Slot0.kI = DriveConstants.STEER_KI;
            steerConfig.Slot0.kD = DriveConstants.STEER_KD;
            steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
            steerMotor.getConfigurator().apply(steerConfig);

            // CANcoder Mutlak Enkoder Ayarları
            absoluteEncoder = new CANcoder(encoderId);
            CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
            absoluteEncoder.getConfigurator().apply(encoderConfig);
        }

        /**
         * Modül için istenen hızı ve açıyı uygular.
         * Dönüşü minimize etmek için ters çevirme senaryolarını düşünür (SwerveModuleState.optimize).
         */
        void setDesiredState(SwerveModuleState desiredState) {
            // CANcoder'dan alınan mevcut yönelim açısı
            double currentAngle = getSteerAngle();
            Rotation2d currentRotation = Rotation2d.fromRotations(currentAngle);

            // Optimizasyon — motorun bir anda 90 dereceden fazla dönmesi yerine 
            // yönü tersine çevirerek çarkı geri çevirir (Tekerleği ters çalıştırır)
            desiredState.optimize(currentRotation);

            // Sürüş: m/s hızını motor (rotor) saniyedeki rotasyon değerine dönüştür
            double driveRPS = desiredState.speedMetersPerSecond
                / DriveConstants.WHEEL_CIRCUMFERENCE
                * DriveConstants.DRIVE_GEAR_RATIO;
            driveMotor.setControl(driveRequest.withVelocity(driveRPS));

            // Yönlendirme: Hangi tur derecesine (pozisyon) bakılacağı belirlenir
            double targetRotations = desiredState.angle.getRotations();
            steerMotor.setControl(steerRequest.withPosition(targetRotations));
        }

        void stop() {
            driveMotor.stopMotor();
            steerMotor.stopMotor();
        }

        /** @return Tur (rotations) cinsinden mevcut dönüş/yönelim açısı (CANcoder'dan). */
        double getSteerAngle() {
            return absoluteEncoder.getAbsolutePosition().getValueAsDouble() - encoderOffset;
        }

        /** @return Modülün metre cinsinden gittiği toplam mesafe ve o anki yönelim açısı. */
        SwerveModulePosition getPosition() {
            double driveRotations = driveMotor.getPosition().getValueAsDouble();
            double distanceMetres = (driveRotations / DriveConstants.DRIVE_GEAR_RATIO)
                * DriveConstants.WHEEL_CIRCUMFERENCE;
            return new SwerveModulePosition(distanceMetres, Rotation2d.fromRotations(getSteerAngle()));
        }
    }
}
