package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Robotun tüm sabit değerlerini (constants) tutan merkezi konfigürasyon
 * dosyası.
 * Anlaşılırlık için alt sistemlere (subsystems) göre düzenlenmiştir.
 * 
 * TODO: Robot mekaniği tamamlandığında ve elektronik montajı bittiğinde
 * buradaki CAN ID'leri, PID değerleri ve fiziksel ölçümler gerçek robotla
 * eşleşecek şekilde GÜNCELLENMELİDİR.
 */
public final class Constants {

    private Constants() {
        // Yardımcı sınıf — örneklenmesini engelle
    }

    // ──────────────────────────────────────────────
    // DRIVETRAIN (Swerve Şasi — MK5n, CTRE Phoenix 6)
    // ──────────────────────────────────────────────
    public static final class DriveConstants {

        // TODO: Pigeon 2.0 IMU CAN ID'sini girin
        public static final int PIGEON_ID = 0;

        // ── Ön Sol Modül (Front-Left) ──
        // TODO: Ön sol modül CAN ID'lerini ve enkoder offset değerini girin
        public static final int FL_DRIVE_ID = 1;
        public static final int FL_STEER_ID = 2;
        public static final int FL_ENCODER_ID = 3;
        public static final double FL_ENCODER_OFFSET = 0.0; // Tur cinsinden (rotations)

        // ── Ön Sağ Modül (Front-Right) ──
        // TODO: Ön sağ modül CAN ID'lerini ve enkoder offset değerini girin
        public static final int FR_DRIVE_ID = 4;
        public static final int FR_STEER_ID = 5;
        public static final int FR_ENCODER_ID = 6;
        public static final double FR_ENCODER_OFFSET = 0.0;

        // ── Arka Sol Modül (Back-Left) ──
        // TODO: Arka sol modül CAN ID'lerini ve enkoder offset değerini girin
        public static final int BL_DRIVE_ID = 7;
        public static final int BL_STEER_ID = 8;
        public static final int BL_ENCODER_ID = 9;
        public static final double BL_ENCODER_OFFSET = 0.0;

        // ── Arka Sağ Modül (Back-Right) ──
        // TODO: Arka sağ modül CAN ID'lerini ve enkoder offset değerini girin
        public static final int BR_DRIVE_ID = 10;
        public static final int BR_STEER_ID = 11;
        public static final int BR_ENCODER_ID = 12;
        public static final double BR_ENCODER_OFFSET = 0.0;

        // Şasi ölçüleri (MK5n için ~0.57 m iz genişliği tipiktir)
        // TODO: Robotun gerçek tekerlekler arası mesafesini inç veya metre cinsinden
        // ölçüp girin
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.5); // metre (sağ-sol arası)
        public static final double WHEEL_BASE = Units.inchesToMeters(22.5); // metre (ön-arka arası)

        // Modüllerin robot merkezine göre konumları
        public static final Translation2d FL_POSITION = new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
        public static final Translation2d FR_POSITION = new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
        public static final Translation2d BL_POSITION = new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
        public static final Translation2d BR_POSITION = new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);

        // MK5n dişli oranı ve tekerlek çapı
        // TODO: Kullanılan swerve dişli oranını (L1, L2, L3) ve teker çapını doğrulayın
        public static final double DRIVE_GEAR_RATIO = 5.357; // L3 konfigürasyonu
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0); // 4 inç tekerlek
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        // Performans sınırları
        // TODO: Robotun maksimum hız testlerini yapıp burayı güncelleyin
        public static final double MAX_SPEED = 5.4; // m/s (L3 için teorik)
        public static final double MAX_ANGULAR_RATE = 2 * Math.PI; // rad/s (saniyede 1 tur dönme)

        // Sürüş motoru PID değerleri (Hız kontrolü)
        // TODO: Sürüş (Drive) motorları için PID ayarlarını (Tuning) yapın
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KV = 0.12;

        // Yönlendirme (Steer) motoru PID değerleri (Pozisyon kontrolü)
        // TODO: Yönlendirme (Steer) motorları için PID ayarlarını yapın
        public static final double STEER_KP = 50.0;
        public static final double STEER_KI = 0.0;
        public static final double STEER_KD = 0.5;
    }

    // ──────────────────────────────────
    // INTAKE (Yerden Alma - 2× NEO)
    // ──────────────────────────────────
    public static final class IntakeConstants {
        // TODO: Intake roller (Pulley) ve açma/kapatma (Pivot - Chain) motorlarının CAN
        // ID'lerini girin
        public static final int ROLLER_MOTOR_ID = 13;
        public static final int PIVOT_MOTOR_ID = 14;

        // ────── ROLLER (Döndürme - Pulley Side) ──────
        // CAD Notu: Roller motorunun ucunda "4:1 MAX Planetary + 90 Derece" modülü ve
        // kayış (pulley) sistemi var.
        // Salt yüzdelik güç (% output) çalıştığı için encoder oranına (gear ratio)
        // kodda ihtiyaç yok.
        // TODO: Bu redüktöre ve kayışa göre ideal lastik (compliant wheel) dönme gücünü
        // (0-1) test edin.
        public static final double INTAKE_SPEED = 0.80; // Yüzdelik güç (0-1)
        public static final int ROLLER_CURRENT_LIMIT = 40; // Amper

        // ────── PIVOT (Açma/Kapatma - Chain Side) MEKANİZMASI ──────
        // CAD Notu: "2x 4:1 (16:1) veya 9:1 MAX Planetary + 90 Derece" modülü + Zincir
        // (Chain) dişli sistemi
        // Zincirde tahminen "am-4773 (küçük) -> am-4779 (büyük)" dişli kullanılıyor.

        // TODO: Intake pivot redüktörünün tam oranını hesaplayıp buraya yazın.
        // Örnek Formül = MAX_Planetary_Oranı * (Büyük_Dişli / Küçük_Dişli)
        // Örnek (16:1 planetary ve 3:1 zincir = Toplam 48.0)
        public static final double PIVOT_GEAR_RATIO = 48.0;

        public static final int PIVOT_CURRENT_LIMIT = 30; // Amper

        // TODO: Pivot (Intake kolu) için PID ayarlarını yapın.
        // Ağırlıktan dolayı (gravity) aşağı inerken hızlı düşmemesi için limit veya PID
        // tuning gerekebilir.
        public static final double PIVOT_KP = 0.1;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.01;

        // TODO: Açık (Deploy/Yerde) ve Kapalı (Stow/Yukarıda) pozisyonlarını redüktör
        // çıkışı (Mekanizma tur sayısı) cinsinden ayarlayın.
        // Örneğin: Kol 90 derece açılıyorsa (Tam 1 turun dörtte biri)
        // PIVOT_DEPLOYED_POSITION = 0.25 olur.
        public static final double PIVOT_DEPLOYED_POSITION = 0.25;
        public static final double PIVOT_STOWED_POSITION = 0.0;
    }

    // ──────────────────────────────────
    // CHAMBER (Ara Bölme - 1× NEO)
    // ──────────────────────────────────
    public static final class ChamberConstants {
        // TODO: Chamber motor CAN ID'sini girin
        public static final int MOTOR_ID = 15;

        // TODO: Oyun parçasını yukarı iletme hızını ayarlayın
        public static final double CHAMBER_SPEED = 0.60; // Yüzdelik güç (0-1)

        public static final int CURRENT_LIMIT = 30; // Amper
    }

    // ──────────────────────────────────────────────────
    // SHOOTER (Atıcı - 3× Vortex + 2× NEO 550 açı ayarı)
    // ──────────────────────────────────────────────────
    public static final class ShooterConstants {

        // Fırlatıcı tekerlek motorları (Flywheel)
        // TODO: Shooter tekerlek motorlarının CAN ID'lerini girin
        public static final int WHEEL_LEADER_ID = 16;
        public static final int WHEEL_FOLLOWER1_ID = 17;
        public static final int WHEEL_FOLLOWER2_ID = 18;

        // TODO: İdeal fırlatma gücünü belirleyin
        public static final double DEFAULT_SHOOTER_SPEED = 1.0; // Yüzdelik güç
        public static final int WHEEL_CURRENT_LIMIT = 60; // Amper

        // Açı ayarlama motorları (Angle)
        // TODO: Açı ayarı yapan motorların CAN ID'lerini girin
        public static final int ANGLE_LEADER_ID = 19;
        public static final int ANGLE_FOLLOWER_ID = 20;

        public static final int ANGLE_CURRENT_LIMIT = 20; // Amper

        // Açı motoru PID değerleri (Pozisyon kontrolü)
        // TODO: Shooter açısı için PID ayarlarını (Tuning) yapın
        public static final double ANGLE_KP = 0.05;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.002;
        public static final double ANGLE_MIN_OUTPUT = -0.5; // Aşağı/yukarı hız sınırı
        public static final double ANGLE_MAX_OUTPUT = 0.5;

        // Açı presetleri (Sıfır noktasından itibaren enkoder dönüş sayısı)
        // TODO: Robot donanımı tamamlandıktan sonra her pozisyon için doğru enkoder
        // değerlerini bulun
        public static final double ANGLE_STOW = 0.0; // Kapalı/İçte
        public static final double ANGLE_SUBWOOFER = 10.0; // Yakın atış
        public static final double ANGLE_PODIUM = 22.0; // Uzak atış
        public static final double ANGLE_AMP = 35.0; // Amp atışı

        // Hedefe ulaşıldı (atSetPoint) kontrolü için hata payı (Tolerans)
        public static final double ANGLE_TOLERANCE = 0.5; // Tur
    }

    // ──────────────────────────────────
    // VISION (Görüntü İşleme - 2× Limelight)
    // ──────────────────────────────────
    public static final class VisionConstants {
        // ──────────────────────────────────
        // FIELD (2026 Saha ve AprilTag Sabitleri)
        // ──────────────────────────────────
        public static final class FieldConstants {
            // İttifaklara göre hedef alınan AprilTag ID dizileri (Örnek: HUB, TOWER vs.)
            public static final int[] RED_TAGS = { 9, 10, 11, 2, 3, 4, 8, 5 };
            public static final int[] BLUE_TAGS = { 19, 20, 21, 24, 18, 27, 26, 25 };
        }

        // TODO: Limelight arayüzünden kameralara verdiğiniz isimleri buraya yazın
        public static final String MAIN_LIMELIGHT = "limelight-main";

        // TODO: Limelight otomatik hizalama (Auto-aim) PID katsayılarını yapın
        // tx değerine göre şasiyi döndürmek için kullanılan PID değerleri
        public static final double AIM_KP = 0.05;
        public static final double AIM_KI = 0.0;
        public static final double AIM_KD = 0.005;
    }

    // ──────────────────────────────────
    // OPERATOR INTERFACE (Kontroller)
    // ──────────────────────────────────
    public static final class OIConstants {
        // Kontrolcü Portları (Driver Station'daki sıra)
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        // Joystick hassasiyeti (küçük titremeleri önlemek için)
        public static final double DEADBAND = 0.08;
    }

    // ──────────────────────────────────
    // FIELD (2026 Saha ve AprilTag Sabitleri)
    // ──────────────────────────────────
    public static final class FieldConstants {
        public static final int[] RED_TAGS = { 9, 10, 11, 2, 3, 4, 8, 5 };
        public static final int[] BLUE_TAGS = { 19, 20, 21, 24, 18, 27, 26, 25 };
    }
}
