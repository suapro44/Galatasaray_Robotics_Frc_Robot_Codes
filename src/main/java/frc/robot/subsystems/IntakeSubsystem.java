package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

/**
 * Yerden alma (Intake) alt sistemi — Leader/Follower (lider/takipçi) konfigürasyonunda
 * iki adet NEO motoru (SparkMax) kullanır. Takipçi, liderin çıkışını otomatik olarak taklit eder.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax leaderMotor;
    private final SparkMax followerMotor;

    public IntakeSubsystem() {
        leaderMotor = new SparkMax(IntakeConstants.LEADER_ID, MotorType.kBrushless);
        followerMotor = new SparkMax(IntakeConstants.FOLLOWER_ID, MotorType.kBrushless);

        // Lider (Leader) motor konfigürasyonu
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig
            .smartCurrentLimit(IntakeConstants.CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);
        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Takipçi (Follower) motor konfigürasyonu — Lideri takip eder
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .smartCurrentLimit(IntakeConstants.CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake)
            .follow(leaderMotor, false);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ────────────── GENEL FONKSİYONLAR (PUBLIC API) ──────────────

    /** Intake motorlarını ayarlanan hızda içeri doğru çalıştırır. */
    public void runIntake() {
        leaderMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    /** Intake motorlarını ters yönde (kusma/eject) çalıştırır. */
    public void reverseIntake() {
        leaderMotor.set(-IntakeConstants.INTAKE_SPEED);
    }

    /** Intake motorlarını durdurur. */
    public void stop() {
        leaderMotor.set(0.0);
    }

    /** @return Intake motorlarının şu an çalışıp çalışmadığı. */
    public boolean isRunning() {
        return Math.abs(leaderMotor.get()) > 0.05;
    }

    // ────────────── PERİYODİK (PERIODIC) ──────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/LeaderOutput", leaderMotor.get());
        SmartDashboard.putNumber("Intake/LeaderCurrent", leaderMotor.getOutputCurrent());
    }
}
