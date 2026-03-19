package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Otomatik Hedef Alma (Auto Aim) Komutu.
 * Şoför robotu X ve Y eksenlerinde (ileri/geri ve sağ/sol) sürmeye devam eder.
 * Ancak dönüş/rotasyon (Z ekseni), Limelight'tan gelen verilere göre otomatik olarak ayarlanır.
 * Eğer Limelight hedef görmüyorsa, şoförün sağ joystick girdisi (manuel kontrol) kullanılır.
 */
public class AutoAimCommand extends Command {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier rotationFallbackSupplier;

    private final PIDController turnPID;

    /**
     * @param drivetrain               Kullanılacak şasi alt sistemi.
     * @param vision                   Limelight verilerini sağlayan görüntü işleme alt sistemi.
     * @param xSpeedSupplier           İleri/geri ekseni (pozitif = ileri).
     * @param ySpeedSupplier           Sağ/sol ekseni (pozitif = sol).
     * @param rotationFallbackSupplier Limelight hedef görmediğinde kullanılacak yedek dönme ekseni.
     */
    public AutoAimCommand(
        DrivetrainSubsystem drivetrain,
        VisionSubsystem vision,
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier rotationFallbackSupplier
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotationFallbackSupplier = rotationFallbackSupplier;

        // Limelight için Dönüş PID kontrolcüsü
        this.turnPID = new PIDController(VisionConstants.AIM_KP, VisionConstants.AIM_KI, VisionConstants.AIM_KD);
        // Hedefimiz Limelight vizöründe tx (yatay sapma) değerini 0 derece yapmak
        this.turnPID.setSetpoint(0.0);

        addRequirements(drivetrain);
        // Vision üzerinde required (zorunluluk) eklememize gerek yok çünkü vision pasif veri okur.
    }

    @Override
    public void execute() {
        // İleri/Geri ve Sağ/Sol (Şoför kontrolü daima devrede)
        double xSpeed = MathUtil.applyDeadband(xSpeedSupplier.getAsDouble(), OIConstants.DEADBAND)
            * DriveConstants.MAX_SPEED;
        double ySpeed = MathUtil.applyDeadband(ySpeedSupplier.getAsDouble(), OIConstants.DEADBAND)
            * DriveConstants.MAX_SPEED;

        double rotationSpeed;

        if (vision.hasMainTarget()) {
            // Hedef görülüyorsa PID ile dönüş hızını hesapla
            // Limelight'ın "tx" degeri hedefin ekran neresinde okundugunu gosterir
            // Eger tx negatifse (hedef solda), robotun sola dönmesi (-yaw) gerekir
            // Not: Drivetrain kinematigi icin rotasyon saatin tersi pozitif, saat yonu negatiftir.
            // PID'den cikan degeri robotun dönme eksenine baglamak icin basina eksi (-) koymak gerekebilir. (Bu donanimsal isleyise gore denenecektir).
            
            double pidOutput = turnPID.calculate(vision.getMainTx(), 0.0);
            
            // Maksimum dönme hizini sinirla (-MAX_ANGULAR_RATE ile +MAX_ANGULAR_RATE arasinda)
            rotationSpeed = MathUtil.clamp(-pidOutput, -DriveConstants.MAX_ANGULAR_RATE, DriveConstants.MAX_ANGULAR_RATE); 
        } else {
            // Hedef görülmüyorsa şoförün sağ analog (fallback) girdisini manuel kullan
            rotationSpeed = MathUtil.applyDeadband(rotationFallbackSupplier.getAsDouble(), OIConstants.DEADBAND)
                * DriveConstants.MAX_ANGULAR_RATE;
        }

        // Saha merkezli sürmeye devam et
        drivetrain.drive(xSpeed, ySpeed, rotationSpeed, true);
    }

    @Override
    public void end(boolean interrupted) {
        // Komut bitince bosta kalmamasi icin durdur
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Tusa basili tutuldugu surece calismaya devam eder
    }
}
