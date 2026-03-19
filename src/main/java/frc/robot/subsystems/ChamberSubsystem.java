package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ChamberConstants;

/**
 * Ara bölme (Chamber / Transfer) alt sistemi — Oyun parçasını intake'ten (yerden alma)
 * alıp shooter'a (fırlatıcı) taşıyan tekli NEO motoru.
 */
public class ChamberSubsystem extends SubsystemBase {

    private final SparkMax motor;

    public ChamberSubsystem() {
        motor = new SparkMax(ChamberConstants.MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(ChamberConstants.CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ────────────── GENEL FONKSİYONLAR (PUBLIC API) ──────────────

    /** Chamber'ı (Ara bölmeyi) ayarlanan hızda ileri doğru çalıştırır. */
    public void runChamber() {
        motor.set(ChamberConstants.CHAMBER_SPEED);
    }

    /** Run the chamber in reverse. */
    public void reverseChamber() {
        motor.set(-ChamberConstants.CHAMBER_SPEED);
    }

    /** Chamber motorunu durdurur. */
    public void stop() {
        motor.set(0.0);
    }

    /** @return Chamber motorunun şu an dönüp dönmediği. */
    public boolean isRunning() {
        return Math.abs(motor.get()) > 0.05;
    }

    // ────────────── PERİYODİK (PERIODIC) ──────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Chamber/Output", motor.get());
        SmartDashboard.putNumber("Chamber/Current", motor.getOutputCurrent());
    }
}
