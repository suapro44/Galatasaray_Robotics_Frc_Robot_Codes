package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;

/**
 * Görüntü İşleme (Vision) alt sistemi — NetworkTables üzerinden iki adet Limelight kamerasıyla haberleşir.
 * <ul>
 *   <li><b>Ana Limelight (Main)</b>  — Saha içi AprilTag'leri ve oyun parçalarını takip eder</li>
 *   <li><b>Tırmanma Limelight (Climb)</b> — Sadece tırmanma hizalaması için AprilTag'leri hedefler</li>
 * </ul>
 *
 * Diğer alt sistemlerin ve komutların (Commands) hizalama verisini sorgulayabilmesi için getter metotları sağlar.
 */
public class VisionSubsystem extends SubsystemBase {

    // ── NetworkTable verileri — Ana Limelight ──
    private final NetworkTableEntry mainTx;
    private final NetworkTableEntry mainTy;
    private final NetworkTableEntry mainTv;

    // ── NetworkTable verileri — Tırmanma Limelight ──
    private final NetworkTableEntry climbTx;
    private final NetworkTableEntry climbTy;
    private final NetworkTableEntry climbTv;

    public VisionSubsystem() {
        NetworkTable mainTable  = NetworkTableInstance.getDefault()
                .getTable(VisionConstants.MAIN_LIMELIGHT);
        NetworkTable climbTable = NetworkTableInstance.getDefault()
                .getTable(VisionConstants.CLIMB_LIMELIGHT);

        mainTx = mainTable.getEntry("tx");
        mainTy = mainTable.getEntry("ty");
        mainTv = mainTable.getEntry("tv");

        climbTx = climbTable.getEntry("tx");
        climbTy = climbTable.getEntry("ty");
        climbTv = climbTable.getEntry("tv");
    }

    // ────────────── ANA KAMERA (MAIN LIMELIGHT) VERİLERİ ──────────────

    /** @return Hedefin merkeze olan yatay sapması (Derece/Degrees). */
    public double getMainTx() {
        return mainTx.getDouble(0.0);
    }

    /** @return Hedefin merkeze olan dikey sapması (Derece/Degrees). */
    public double getMainTy() {
        return mainTy.getDouble(0.0);
    }

    /** @return Ana Limelight'ın geçerli bir hedef görüp görmediği (1.0 = Evet). */
    public boolean hasMainTarget() {
        return mainTv.getDouble(0.0) >= 1.0;
    }

    // ────────────── TIRMANMA KAMERASI (CLIMB LIMELIGHT) VERİLERİ ──────────────

    /** @return Hedefin merkeze olan yatay sapması (Derece). */
    public double getClimbTx() {
        return climbTx.getDouble(0.0);
    }

    /** @return Hedefin merkeze olan dikey sapması (Derece). */
    public double getClimbTy() {
        return climbTy.getDouble(0.0);
    }

    /** @return Tırmanma Limelight'ının geçerli bir hedef görüp görmediği. */
    public boolean hasClimbTarget() {
        return climbTv.getDouble(0.0) >= 1.0;
    }

    // ────────────── BORU HATTI (PIPELINE) KONTROLÜ ──────────────

    /**
     * Ana Limelight üzerinde görüntü işleme boru hattını (pipeline) değiştirir.
     *
     * @param pipeline Boru hattı indeksi (0–9).
     */
    public void setMainPipeline(int pipeline) {
        NetworkTableInstance.getDefault()
            .getTable(VisionConstants.MAIN_LIMELIGHT)
            .getEntry("pipeline")
            .setNumber(pipeline);
    }

    /**
     * Tırmanma Limelight üzerinde görüntü işleme boru hattını (pipeline) değiştirir.
     *
     * @param pipeline Boru hattı indeksi (0–9).
     */
    public void setClimbPipeline(int pipeline) {
        NetworkTableInstance.getDefault()
            .getTable(VisionConstants.CLIMB_LIMELIGHT)
            .getEntry("pipeline")
            .setNumber(pipeline);
    }

    // ────────────── PERİYODİK (PERIODIC) ──────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vision/Main/tx", getMainTx());
        SmartDashboard.putNumber("Vision/Main/ty", getMainTy());
        SmartDashboard.putBoolean("Vision/Main/HasTarget", hasMainTarget());

        SmartDashboard.putNumber("Vision/Climb/tx", getClimbTx());
        SmartDashboard.putNumber("Vision/Climb/ty", getClimbTy());
        SmartDashboard.putBoolean("Vision/Climb/HasTarget", hasClimbTarget());
    }
}
