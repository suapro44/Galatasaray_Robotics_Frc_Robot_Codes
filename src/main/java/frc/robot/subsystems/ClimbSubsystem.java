package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;

/**
 * Tırmanma (Climb) alt sistemi — PID pozisyon kontrollü, dâhili relatif
 * enkoder kullanan tekli NEO motoru (SparkMax). Tek tuşla Seviye 3'e tırmanma sağlar.
 */
public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    private double targetPosition = ClimbConstants.STOW_SETPOINT;

    public ClimbSubsystem() {
        motor = new SparkMax(ClimbConstants.MOTOR_ID, MotorType.kBrushless);

        // Motoru PID ile yapılandır
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(ClimbConstants.CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);
        config.closedLoop
            .p(ClimbConstants.CLIMB_KP)
            .i(ClimbConstants.CLIMB_KI)
            .d(ClimbConstants.CLIMB_KD)
            .outputRange(ClimbConstants.CLIMB_MIN_OUTPUT, ClimbConstants.CLIMB_MAX_OUTPUT);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Relatif enkoder — Başlangıç noktasını sıfır kabul eder (tamamen aşağıda/kapalı olduğunu varsayar)
        encoder = motor.getEncoder();
        encoder.setPosition(0.0);

        // Kapalı çevrim (Closed-loop) kontrolcüsü
        pidController = motor.getClosedLoopController();
    }

    // ────────────── GENEL FONKSİYONLAR (PUBLIC API) ──────────────

    /**
     * Tırmanıcıya Seviye 3 yüksekliğine çıkmasını emreder.
     */
    public void climbToLevel3() {
        setPosition(ClimbConstants.LEVEL_3_SETPOINT);
    }

    /**
     * Tırmanıcıyı toplama (stow) / kapalı pozisyonuna indirir.
     */
    public void stow() {
        setPosition(ClimbConstants.STOW_SETPOINT);
    }

    /**
     * Tırmanıcıyı istenilen herhangi bir pozisyona ayarlar.
     *
     * @param position Enkoder tur (rotations) cinsinden hedef pozisyon.
     */
    public void setPosition(double position) {
        this.targetPosition = position;
        pidController.setReference(position, ControlType.kPosition);
    }

    /** Tırmanma motorunu durdurur (Brake modu sayesinde olduğu yerde kalır). */
    public void stop() {
        motor.set(0.0);
    }

    /**
     * @return Tırmanıcının hedef pozisyona (hata payı içerisinde) ulaşıp ulaşmadığı.
     */
    public boolean isAtTarget() {
        return Math.abs(encoder.getPosition() - targetPosition) < ClimbConstants.POSITION_TOLERANCE;
    }

    /** @return Güncel enkoder pozisyonu (Tur/Rotations cinsinden). */
    public double getCurrentPosition() {
        return encoder.getPosition();
    }

    // ────────────── PERİYODİK (PERIODIC) ──────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/Position", encoder.getPosition());
        SmartDashboard.putNumber("Climb/Target", targetPosition);
        SmartDashboard.putBoolean("Climb/AtTarget", isAtTarget());
        SmartDashboard.putNumber("Climb/Current", motor.getOutputCurrent());
    }
}
