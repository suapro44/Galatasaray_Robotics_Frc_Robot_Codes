package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Tırmanma (Climb) mekanizması için AprilTag'e otomatik konumlanma (Auto-Align) komutu.
 * Bu komut "climb" Limelight kamerasını kullanarak robotu:
 * 1. tx (yatay açı) verisi ile yatayda döndürerek hedefe bakar hale getirir.
 * 2. ty (dikey açı) verisi ile ileri/geri giderek doğru mesafede durmasını sağlar.
 */
public class AutoClimbAlignCommand extends Command {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;

    // Robotun dönüş (rotation) hassasiyetini yönetecek PID (tx kullanır)
    private final PIDController turnPID;
    
    // Robotun ileri-geri mesafesini (distance) yönetecek PID (ty kullanır)
    private final PIDController distancePID;

    public AutoClimbAlignCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Dönüş (tx) PID Ayarları
        this.turnPID = new PIDController(VisionConstants.CLIMB_ALIGN_KP, 0.0, VisionConstants.CLIMB_ALIGN_KD);
        this.turnPID.setSetpoint(0.0); // Hedef tx'i sıfırlamak (tam karşıya bakmak)

        // Mesafe (ty) PID Ayarları
        this.distancePID = new PIDController(VisionConstants.CLIMB_DISTANCE_KP, 0.0, VisionConstants.CLIMB_DISTANCE_KD);
        this.distancePID.setSetpoint(VisionConstants.TARGET_TY_FOR_CLIMB); // İdeal tırmanma mesafesindeki ty hedefi

        // Şasi kontrolünü bizim komutumza alması için required (zorunlu) ekliyoruz
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!vision.hasClimbTarget()) {
            // Limelight hedef AprilTag'i kaybederse veya göremiyorsa motorları güvenliğe al ve durdur.
            drivetrain.stop();
            return;
        }

        // 1) DÖNÜŞ (TX): tx değerine göre dönüş (rotation) hızını hesapla
        // Eğer tx pozitifse (robotun sağına düşüyorsa), sağa dön (negatif rotasyon hızı).
        double rotationOutput = turnPID.calculate(vision.getClimbTx(), 0.0);
        
        // Robotun maksimum dönebilme hızının %30'undan fazlasıyla dönmesini engelle (güvenlik için)
        double rotationSpeed = MathUtil.clamp(-rotationOutput, -DriveConstants.MAX_ANGULAR_RATE * 0.3, DriveConstants.MAX_ANGULAR_RATE * 0.3);

        // 2) MESAFE (TY): ty değerine göre ileri/geri (translation X) hızını hesapla
        // Hedeflenen TARGET_TY_FOR_CLIMB noktasından sapmaya göre şasiye xSpeed uygula.
        // PID'nin eksi veya artı işareti kameranın montaj açısına göre değişkenlik gösterdiğinden test ederek başına '-' konabilir.
        double distanceOutput = distancePID.calculate(vision.getClimbTy(), VisionConstants.TARGET_TY_FOR_CLIMB);
        
        // Arabanın aniden hızlanmasını önlemek adına hızı maksimum sürüş hızının %25'i ile sınırla.
        double forwardSpeed = MathUtil.clamp(distanceOutput, -DriveConstants.MAX_SPEED * 0.25, DriveConstants.MAX_SPEED * 0.25);

        // Şasiye komutu ilet: Yana doğru kayma (ySpeed) yok, sadece ileri/geri (X) ve kendi etrafında dönüş (Z).
        // false parametresi "Robot Merkezli Mode (Robot Centric)" içindir çünkü kameranın ve robotun kendi bakış açısına göre gidiyoruz.
        drivetrain.drive(forwardSpeed, 0.0, rotationSpeed, false);
    }

    @Override
    public void end(boolean interrupted) {
        // Tuş bırakıldığında veya komut kesildiğinde robotu frenlet.
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        // D-Pad tuşuna basılı tutulduğu sürece devam etmeli. Otonom olarak bitmesi isteniyorsa buraya PID tolerans checki eklenebilir.
        return false;
    }
}
