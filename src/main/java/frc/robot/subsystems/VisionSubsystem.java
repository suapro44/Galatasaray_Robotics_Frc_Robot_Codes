package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import java.io.UncheckedIOException;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    
    private final NetworkTable limelightTable;
    private AprilTagFieldLayout fieldLayout;

    // Kırmızı ve Mavi ittifak için hedef tag dizileri
    private final int[] redAllianceTags = {9, 10, 11, 2, 3, 4, 8, 5};
    private final int[] blueAllianceTags = {19, 20, 21, 24, 18, 27, 26, 25};

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight-main");
        
        try {
            // WPILib 2026 içerisinde tanımlanması beklenen REBUILT varsayılan sahasını yükler.
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (UncheckedIOException e) {
            DriverStation.reportError("2026 (REBUILT) saha haritası yüklenemedi: " + e.getMessage(), false);
            fieldLayout = null;
        }
    }

    /**
     * NetworkTables üzerinden limelight-main kamerasından botpose_wpiblue verisini okuyup Pose2d olarak döndürür.
     * 
     * @return Robotun sahadaki güncel Pose2d konumu
     */
    public Pose2d getBotPoseWpiBlue() {
        double[] defaultPose = new double[6];
        double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(defaultPose);

        if (botpose.length >= 6) {
            return new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
        }
        return new Pose2d();
    }

    /**
     * DriverStation üzerinden robotun ittifak rengini kontrol eder.
     * Girilen bir indekse (0-7) göre doğru Tag ID'sini döndürür.
     * 
     * @param index Talep edilen tagın dizideki indeksi (0-7 arası)
     * @return İttifak rengine uygun Tag ID'si (Geçersiz indeks için -1 döndürür)
     */
    public int getTargetTagId(int index) {
        if (index < 0 || index > 7) {
            return -1;
        }

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        
        // Eğer ittifak Kırmızı ise kırmızı dizisindeki değeri id olarak ver
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return redAllianceTags[index];
        } 
        
        // Varsayılan veya ittifak Mavi ise mavi dizinden ver
        return blueAllianceTags[index];
    }

    /**
     * İttifak kontrolcüsünden dönen Tag ID'sinin sahadaki 3D (X,Y) koordinatlarını haritadan çekip, 
     * robotun güncel BotPose (X,Y) konumu ile arasındaki mesafeyi metre cinsinden hesaplar.
     * 
     * @param index Talep edilen tagın dizideki indeksi (0-7)
     * @return Robot ile hedef Tag arasındaki mesafe (Metre). Hedef veya harita yoksa -1 döndürür.
     */
    public double getDistanceToTag(int index) {
        // Harita yoksa mesafe hesaplanamaz
        if (fieldLayout == null) {
            return -1.0;
        }

        // Limelight herhangi bir hedef göremiyorsa (tv = 0), güncel pozisyon güvenilir olmayabilir
        if (limelightTable.getEntry("tv").getDouble(0.0) == 0.0) {
            return -1.0;
        }

        int tagId = getTargetTagId(index);
        
        // Indeks geçersizse mesafe hesaplanamaz
        if (tagId == -1) {
            return -1.0;
        }

        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);
        
        // Harita içerisinde hedef Tag bulunamazsa mesafe hesaplanamaz
        if (tagPoseOpt.isEmpty()) {
            return -1.0;
        }

        // Tag 3D konum ve Robot 2D konum verilerini al
        Pose3d tagPose = tagPoseOpt.get();
        Pose2d botPose = getBotPoseWpiBlue();

        // 2D düzlemdeki X ve Y noktaları üzerinden Translation2d oluşturulur 
        Translation2d tagTranslation = new Translation2d(tagPose.getX(), tagPose.getY());
        Translation2d botTranslation = botPose.getTranslation();

        // İki koordinat arasındaki metrik mesafeyi döndür
        return botTranslation.getDistance(tagTranslation);
    }

    // --- Önceki koddaki eksik Limelight methodları (Derleme Hatalarını Çözmek İçin) --- //

    public boolean hasMainTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getMainTx() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    // Tırmanma Limelight işlemleri için
    private final NetworkTable climbTable = NetworkTableInstance.getDefault().getTable("limelight-climb");

    public boolean hasClimbTarget() {
        return climbTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getClimbTx() {
        return climbTable.getEntry("tx").getDouble(0.0);
    }

    public double getClimbTy() {
        return climbTable.getEntry("ty").getDouble(0.0);
    }
}
