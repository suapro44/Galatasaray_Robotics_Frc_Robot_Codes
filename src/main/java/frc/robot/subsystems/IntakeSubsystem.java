package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
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
 * Yerden alma (Intake) alt sistemi — Roller ve mafsal (Pivot/Açma-Kapatma) motorlarını içerir.
 * Roller oyun parçasını emerken, Pivot motoru (PID ve redüktör ayarlı) intake'i indirir/kaldırır.
 */
public class IntakeSubsystem extends SubsystemBase {

    // ── Motorlar ──
    private final SparkMax rollerMotor;
    private final SparkMax pivotMotor;

    // ── Pivot Kontrolü ──
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotPID;

    private double pivotTargetPosition = IntakeConstants.PIVOT_STOWED_POSITION;

    public IntakeSubsystem() {
        rollerMotor = new SparkMax(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
        pivotMotor  = new SparkMax(IntakeConstants.PIVOT_MOTOR_ID,  MotorType.kBrushless);

        // ────── Roller (Döndürme) Konfigürasyonu ──────
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
            .smartCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ────── Pivot (Açma/Kapatma) Konfigürasyonu ve PID ──────
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig
            .smartCurrentLimit(IntakeConstants.PIVOT_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);
            
        // Redüktör Oranını (Gearbox Ratio) enkoder faktörü olarak ata
        pivotConfig.encoder
            .positionConversionFactor(1.0 / IntakeConstants.PIVOT_GEAR_RATIO)
            .velocityConversionFactor(1.0 / IntakeConstants.PIVOT_GEAR_RATIO);

        pivotConfig.closedLoop
            .p(IntakeConstants.PIVOT_KP)
            .i(IntakeConstants.PIVOT_KI)
            .d(IntakeConstants.PIVOT_KD);
            
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Enkoder işlemleri
        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPosition(0.0); // Robot açılışında kapalı (0) olduğunu varsay

        pivotPID = pivotMotor.getClosedLoopController();
    }

    // ────────────── ROLLER (DÖNDÜRME) API ──────────────

    /** Intake roller motorunu ayarlanan hızda içeri doğru çalıştırır. */
    public void runIntake() {
        rollerMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    /** Intake motorunu ters yönde (kusma/eject) çalıştırır. */
    public void reverseIntake() {
        rollerMotor.set(-IntakeConstants.INTAKE_SPEED);
    }

    /** Intake roller motorunu durdurur. */
    public void stop() {
        rollerMotor.set(0.0);
    }

    /** @return Intake roller motorunun şu an çalışıp çalışmadığı. */
    public boolean isRunning() {
        return Math.abs(rollerMotor.get()) > 0.05;
    }

    // ────────────── PIVOT (AÇMA/KAPATMA) API ──────────────

    /** Intake'i yere/almaya hazır hale getirir (İndirir). */
    public void deploy() {
        this.pivotTargetPosition = IntakeConstants.PIVOT_DEPLOYED_POSITION;
        pivotPID.setReference(IntakeConstants.PIVOT_DEPLOYED_POSITION, ControlType.kPosition);
    }

    // Intake'i koruma pozisyonuna geri çeker (Kaldırır). 
    public void retract() {
        this.pivotTargetPosition = IntakeConstants.PIVOT_STOWED_POSITION;
        pivotPID.setReference(IntakeConstants.PIVOT_STOWED_POSITION, ControlType.kPosition);
    }

    // Intake'i belli bir hedefe ayarlar (Eğer özelleşmiş bir derece istenirse)
    public void setPivotPosition(double position) {
        this.pivotTargetPosition = position;
        pivotPID.setReference(position, ControlType.kPosition);
    }

    // ────────────── PERİYODİK (PERIODIC) ──────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/RollerOutput", rollerMotor.get());
        SmartDashboard.putNumber("Intake/PivotPosition", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Intake/PivotTarget", pivotTargetPosition);
        SmartDashboard.putNumber("Intake/PivotCurrent", pivotMotor.getOutputCurrent());
    }
}
