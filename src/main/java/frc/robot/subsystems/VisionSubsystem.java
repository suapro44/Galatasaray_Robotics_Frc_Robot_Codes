package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import java.io.UncheckedIOException;
import java.util.Optional;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants;

public class VisionSubsystem extends SubsystemBase {

    // ── NetworkTable verileri — Ana Limelight ──
    private final NetworkTableEntry mainTx;
    private final NetworkTableEntry mainTy;
    private final NetworkTableEntry mainTv;
    private final NetworkTableEntry mainBotPoseBlue; // YENİ: 3D Konum

    // YENİ: Saha Haritası
    private AprilTagFieldLayout fieldLayout;

    public VisionSubsystem() {
        NetworkTable mainTable  = NetworkTableInstance.getDefault().getTable(VisionConstants.MAIN_LIMELIGHT);

        mainTx = mainTable.getEntry("tx");
        mainTy = mainTable.getEntry("ty");
        mainTv = mainTable.getEntry("tv");
        mainBotPoseBlue = mainTable.getEntry("botpose_wpiblue"); // 3D MegaTag verisi

        // YENİ: 2026 Varsayılan Saha Haritasını Yükleme
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (UncheckedIOException e) {
            System.out.println("Saha haritası yüklenemedi! WPILib sürümünü kontrol et.");
        }
    }

    // ────────────── ANA KAMERA (MAIN LIMELIGHT) 2D VERİLERİ ──────────────
    public double getMainTx() { return mainTx.getDouble(0.0); }
    public double getMainTy() { return mainTy.getDouble(0.0); }
    public boolean hasMainTarget() { return mainTv.getDouble(0.0) >= 1.0; }

    // ────────────── YENİ: 3D MEGA TAG VE MESAFE KONTROLÜ ──────────────

    /** Limelight'tan robotun Mavi İttifak orijinine göre (X,Y) konumunu çeker. */
    public Pose2d getBotPoseBlue() {
        double[] poseArray = mainBotPoseBlue.getDoubleArray(new double[6]);
        if (poseArray.length >= 6) {
            return new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
        }
        return new Pose2d();
    }

    /** İttifak rengine göre hedef AprilTag ID'sini verir. */
    public int getTargetTagId(int targetIndex) {
        if (targetIndex < 0 || targetIndex >= FieldConstants.RED_TAGS.length) return -1;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return FieldConstants.RED_TAGS[targetIndex];
        }
        return FieldConstants.BLUE_TAGS[targetIndex];
    }

    /** Robotun hedef Tag'e olan mesafesini (Metre) hesaplar. */
    public double getDistanceToTarget(int targetTagId) {
        if (!hasMainTarget() || fieldLayout == null) return -1.0; 

        Pose2d robotPose = getBotPoseBlue();
        var tagPoseOptional = fieldLayout.getTagPose(targetTagId);
        
        if (tagPoseOptional.isPresent()) {
            Pose2d tagPose2d = tagPoseOptional.get().toPose2d();
            return robotPose.getTranslation().getDistance(tagPose2d.getTranslation());
        }
        return -1.0;
    }

    // ────────────── BORU HATTI (PIPELINE) KONTROLÜ ──────────────
    public void setMainPipeline(int pipeline) {
        NetworkTableInstance.getDefault().getTable(VisionConstants.MAIN_LIMELIGHT).getEntry("pipeline").setNumber(pipeline);
    }

    // ────────────── PERİYODİK (PERIODIC) ──────────────
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vision/Main/tx", getMainTx());
        SmartDashboard.putNumber("Vision/Main/ty", getMainTy());
        SmartDashboard.putBoolean("Vision/Main/HasTarget", hasMainTarget());
        
        // Mesafe test çıktısı (Örn: 0. indeksteki ana hedef)
        int currentTargetId = getTargetTagId(0);
        SmartDashboard.putNumber("Vision/Main/DistanceMeters", getDistanceToTarget(currentTargetId));
    }
}