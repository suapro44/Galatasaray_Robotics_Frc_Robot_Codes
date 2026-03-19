package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

/**
 * Shooter (Atıcı/Fırlatıcı) alt sistemi şunları içerir:
 * <ul>
 *   <li>3× Vortex (SparkFlex) — Fırlatıcı tekerlekleri (1 Lider + 2 Takipçi)</li>
 *   <li>2× NEO 550 (SparkMax) — Açı ayarlama mekanizması (1 Lider + 1 Takipçi)</li>
 * </ul>
 *
 * Açı mekanizması, NEO 550 dâhili (relatif) kodlayıcısı (encoder) ve 
 * SparkClosedLoopController üzerinden pozisyon modunda kontrol edilir.
 * Mekanizma maç başladığında en alt konumda (0 değeri) kabul edilir.
 */
public class ShooterSubsystem extends SubsystemBase {

    // ── Fırlatıcı Tekerlek motorları ──
    private final SparkFlex wheelLeader;
    private final SparkFlex wheelFollower1;
    private final SparkFlex wheelFollower2;

    // ── Açı Kontrol Motorları ──
    private final SparkMax angleLeader;
    private final SparkMax angleFollower;

    // ── Açı Kontrol Mekanizması ──
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;

    private double angleSetpoint = ShooterConstants.ANGLE_STOW;

    public ShooterSubsystem() {
        // ─────── Fırlatıcı Tekerlek (Flywheel) Kurulumu ───────
        wheelLeader    = new SparkFlex(ShooterConstants.WHEEL_LEADER_ID,    MotorType.kBrushless);
        wheelFollower1 = new SparkFlex(ShooterConstants.WHEEL_FOLLOWER1_ID, MotorType.kBrushless);
        wheelFollower2 = new SparkFlex(ShooterConstants.WHEEL_FOLLOWER2_ID, MotorType.kBrushless);

        // Lider motor konfigürasyonu
        SparkFlexConfig wheelLeaderConfig = new SparkFlexConfig();
        wheelLeaderConfig
            .smartCurrentLimit(ShooterConstants.WHEEL_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast); // Atış sonrası serbest dönmesi için Coast modu
        wheelLeader.configure(wheelLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Takipçi 1 konfigürasyonu
        SparkFlexConfig follower1Config = new SparkFlexConfig();
        follower1Config
            .smartCurrentLimit(ShooterConstants.WHEEL_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast)
            .follow(wheelLeader, false);
        wheelFollower1.configure(follower1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Takipçi 2 konfigürasyonu
        SparkFlexConfig follower2Config = new SparkFlexConfig();
        follower2Config
            .smartCurrentLimit(ShooterConstants.WHEEL_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast)
            .follow(wheelLeader, false);
        wheelFollower2.configure(follower2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ─────── Açı Mekanizması (Angle) Kurulumu ───────
        angleLeader   = new SparkMax(ShooterConstants.ANGLE_LEADER_ID,   MotorType.kBrushless);
        angleFollower = new SparkMax(ShooterConstants.ANGLE_FOLLOWER_ID, MotorType.kBrushless);

        // PID destekli Lider Açı Motoru konfigürasyonu
        SparkMaxConfig angleLeaderConfig = new SparkMaxConfig();
        angleLeaderConfig
            .smartCurrentLimit(ShooterConstants.ANGLE_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake); // Olduğu yerde kalması için Brake modu şart
        angleLeaderConfig.closedLoop
            .p(ShooterConstants.ANGLE_KP)
            .i(ShooterConstants.ANGLE_KI)
            .d(ShooterConstants.ANGLE_KD)
            .outputRange(ShooterConstants.ANGLE_MIN_OUTPUT, ShooterConstants.ANGLE_MAX_OUTPUT);
        angleLeader.configure(angleLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Takipçi Açı Motoru konfigürasyonu
        SparkMaxConfig angleFollowerConfig = new SparkMaxConfig();
        angleFollowerConfig
            .smartCurrentLimit(ShooterConstants.ANGLE_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake)
            .follow(angleLeader, false);
        angleFollower.configure(angleFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Relatif (Dâhili) Enkoder — Robot başladığında pozisyon sıfır kabul edilir (en alt konum)
        angleEncoder = angleLeader.getEncoder();
        angleEncoder.setPosition(0.0); // Başlangıçta sıfırla

        // Kapalı Çevrim (PID) Kontrolcüsü
        anglePID = angleLeader.getClosedLoopController();
    }

    // ────────────── FIRLATICI TEKERLEK (FLYWHEEL) API ──────────────

    /**
     * Fırlatıcı tekerlek çalışma hızını ayarlar.
     *
     * @param speed Yüzdelik güç (0.0 = durmuş, 1.0 = tam güç ileri).
     */
    public void setShooterSpeed(double speed) {
        wheelLeader.set(speed);
    }

    /** Fırlatıcıyı Constants dosyasında belirlenen varsayılan güçte çalıştırır. */
    public void runShooter() {
        setShooterSpeed(ShooterConstants.DEFAULT_SHOOTER_SPEED);
    }

    /** Fırlatıcı motorları durdurur (Motorlar coast -serbest- modunda yavaşlayarak duracaktır). */
    public void stopShooter() {
        wheelLeader.set(0.0);
    }

    // ────────────── AÇI (ANGLE) API ──────────────

    /**
     * Shooter açısını belirtilen pozisyon hedefine göderir (PID Kullanarak).
     *
     * @param setpoint Sıfır noktasından itibaren enkoder dönüş sayısı cinsinden hedef pozisyon.
     */
    public void setAngle(double setpoint) {
        this.angleSetpoint = setpoint;
        anglePID.setReference(setpoint, ControlType.kPosition);
    }

    /** Açıyı kapalı (Stow/Sıfır) pozisyona gönderir. */
    public void stowAngle() {
        setAngle(ShooterConstants.ANGLE_STOW);
    }

    /** Açı motorlarını durdurur (Motorlar Brake modunda olduğu için olduğu yerde asılı kalır). */
    public void stopAngle() {
        angleLeader.set(0.0);
    }

    /**
     * @return Açı mekanizmasının PID hedefine yeterince (hata payı içinde) yaklaşıp yaklaşmadığını döndürür.
     */
    public boolean isAtTargetAngle() {
        return Math.abs(angleEncoder.getPosition() - angleSetpoint) < ShooterConstants.ANGLE_TOLERANCE;
    }

    /** @return Güncel enkoder pozisyonu (Tur cinsinden). */
    public double getCurrentAngle() {
        return angleEncoder.getPosition();
    }

    // ────────────── PERİYODİK (PERIODIC) ──────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/WheelOutput", wheelLeader.get());
        SmartDashboard.putNumber("Shooter/WheelCurrent", wheelLeader.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/AnglePosition", angleEncoder.getPosition());
        SmartDashboard.putNumber("Shooter/AngleSetpoint", angleSetpoint);
        SmartDashboard.putBoolean("Shooter/AtTargetAngle", isAtTargetAngle());
    }
}
