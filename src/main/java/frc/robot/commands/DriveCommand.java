package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Sürücü komutu (Default Drive Command). Joystick girdilerini okuyarak 
 * Swerve şasiyi saha merkezli (field-centric) modda sürekli olarak kontrol eder.
 */
public class DriveCommand extends Command {

    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier rotationSupplier;

    /**
     * @param drivetrain       Kullanılacak şasi alt sistemi.
     * @param xSpeedSupplier   İleri/geri ekseni (pozitif = ileri).
     * @param ySpeedSupplier   Sağ/sol ekseni (pozitif = sol).
     * @param rotationSupplier Dönme ekseni (pozitif = saat yönünün tersine dönüş).
     */
    public DriveCommand(
        DrivetrainSubsystem drivetrain,
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier rotationSupplier
    ) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Joystickteki küçük titremeleri yoksay (Deadband uygula) ve max hızla çarp
        double xSpeed = MathUtil.applyDeadband(xSpeedSupplier.getAsDouble(), OIConstants.DEADBAND)
            * DriveConstants.MAX_SPEED;
        double ySpeed = MathUtil.applyDeadband(ySpeedSupplier.getAsDouble(), OIConstants.DEADBAND)
            * DriveConstants.MAX_SPEED;
        double rotation = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), OIConstants.DEADBAND)
            * DriveConstants.MAX_ANGULAR_RATE;

        drivetrain.drive(xSpeed, ySpeed, rotation, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Başka bir komut tarafından bölünene kadar sürekli çalışır (Varsayılan komut mantığı)
    }
}
