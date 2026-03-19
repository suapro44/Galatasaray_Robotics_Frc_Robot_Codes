package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoClimbAlignCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ChamberSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Bu sınıf, fiziksel operatör arayüzü (kontrolcüler) ile komutlar/alt sistemler
 * arasındaki bağlantıların (bağlayıcı, "glue") kurulduğu yerdir. Alt sistemler
 * de burada tanımlanır (instantiate edilir).
 *
 * <h2>Kontrolcü Düzeni (Xbox)</h2>
 * <table>
 *   <tr><td>Sol Analog (Left Stick)</td><td>Saha merkezli sürüş (X/Y)</td></tr>
 *   <tr><td>Sağ Analog X (Right Stick X)</td><td>Dönüş / Rotasyon</td></tr>
 *   <tr><td>Start Tuşu</td><td>Gyro açısını sıfırlar (İleri yönü belirler)</td></tr>
 *   <tr><td>Sağ Tampon (RB)</td><td>Yerden alma - Intake (Basılı tut)</td></tr>
 *   <tr><td>Sol Tampon (LB)</td><td>Ara bölme - Chamber ileri (Basılı tut)</td></tr>
 *   <tr><td>Sağ Tetik (RT)</td><td>Ateş etme sekansı (Motorları hızlandır + parçayı ilet)</td></tr>
 *   <tr><td>Sol Tetik (LT)</td><td>Limelight ile Otomatik Hedef Alma (Auto-Aim)</td></tr>
 *   <tr><td>A Tuşu</td><td>Açı ayarı (Toplanmış pozisyon - Stow)</td></tr>
 *   <tr><td>B Tuşu</td><td>Intake ters (Kusma - Eject)</td></tr>
 *   <tr><td>X Tuşu</td><td>Açı ayarı (Podium / Uzak atış)</td></tr>
 *   <tr><td>Y Tuşu</td><td>Seviye 3'e tırman</td></tr>
 *   <tr><td>D-Pad Yukarı</td><td>Açı ayarı (Subwoofer / Yakın atış)</td></tr>
 *   <tr><td>D-Pad Aşağı</td><td>Tırmanma AprilTag Hizalaması (Auto Climb Align)</td></tr>
 * </table>
 */
public class RobotContainer {

    // ── Alt Sistemler (Subsystems) ──
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final IntakeSubsystem     intake     = new IntakeSubsystem();
    private final ChamberSubsystem    chamber    = new ChamberSubsystem();
    private final ShooterSubsystem    shooter    = new ShooterSubsystem();
    private final ClimbSubsystem      climb      = new ClimbSubsystem();
    private final VisionSubsystem     vision     = new VisionSubsystem();

    // ── Kontrolcüler (Controllers) ──
    private final CommandXboxController driverController =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    // ────────────── VARSAYILAN KОMUTLAR (DEFAULT COMMANDS) ──────────────

    private void configureDefaultCommands() {
        // Varsayılan sürüş komutu — Sol analog hareket (translation), sağ analog X yatay dönüş (rotation) içindir.
        // Not: Joysticklerde ileri itildiğinde negatif değer geldiği için Y ekseni eksi ile çarpılır (-driverController.getLeftY()).
        drivetrain.setDefaultCommand(
            new DriveCommand(
                drivetrain,
                () -> -driverController.getLeftY(),   // forward/backward
                () -> -driverController.getLeftX(),   // left/right strafe
                () -> -driverController.getRightX()   // rotation
            )
        );
    }

    // ────────────── TUŞ ATAMALARI (BUTTON BINDINGS) ──────────────

    private void configureButtonBindings() {

        // ── Start Tuşu: Gyro'yu sıfırlar (Saha Merkezli sürüş için robotun baktığı yönü İLERİ kabul eder) ──
        driverController.start().onTrue(
            Commands.runOnce(() -> drivetrain.zeroHeading(), drivetrain)
        );

        // ── Sağ Tampon (RB): Intake (Basılı tutulduğunda çalışır, bırakıldığında durur) ──
        driverController.rightBumper()
            .whileTrue(Commands.startEnd(
                () -> {
                    intake.deploy();
                    intake.runIntake();
                },
                () -> {
                    intake.retract();
                    intake.stop();
                },
                intake
            ));

        // ── B Tuşu: Ters Intake / Kusma (Basılı tutulduğunda çalışır) ──
        driverController.b()
            .whileTrue(Commands.startEnd(
                () -> {
                    intake.deploy();
                    intake.reverseIntake();
                },
                () -> {
                    intake.retract();
                    intake.stop();
                },
                intake
            ));

        // ── Sol Tampon (LB): Ara Bölme (Chamber) İleri (Basılı tut) ──
        driverController.leftBumper()
            .whileTrue(Commands.startEnd(
                chamber::runChamber,
                chamber::stop,
                chamber
            ));

        // ── Sağ Tetik (RT > 0.5): Atış Sekansı ──
        //    1. Fırlatıcı tekerlekleri hızlandır (Spin up)
        //    2. Yeterli süre bekle
        //    3. Ara bölmeyi (Chamber) çalıştırarak parçayı tekerleklere ilet
        //    Tetik bırakıldığında her şeyi durdur
        driverController.rightTrigger(0.5)
            .whileTrue(
                Commands.parallel(
                    // Fırlatıcıyı başlat
                    Commands.startEnd(
                        shooter::runShooter,
                        shooter::stopShooter,
                        shooter
                    ),
                    // Hızlanma için kısa bir gecikme (0.5 sn) sonrası chamber'ı besle
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        Commands.startEnd(
                            chamber::runChamber,
                            chamber::stop,
                            chamber
                        )
                    )
                )
            );

        // ── Sol Tetik (LT > 0.5): Otomatik Hedef Alma (Auto-Aim) ──
        // Şoför LT'ye basılı tutarken sürüşe (ileri/geri ve sağ/sol) devam edebilir.
        // Limelight hedefi gördüğünde sağ/sol joystickten aldığı dönme(rotasyon) etkisini ezer.
        driverController.leftTrigger(0.5)
            .whileTrue(new AutoAimCommand(
                drivetrain,
                vision,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()
            ));

        // ── D-Pad Yukarı (POV 0): Açı - Subwoofer (Yakın Atış) Preset'i ──
        driverController.pov(0)
            .onTrue(Commands.runOnce(
                () -> shooter.setAngle(ShooterConstants.ANGLE_SUBWOOFER), shooter
            ));

        // ── D-Pad Aşağı (POV 180): Otomatik Tırmanma Hizalaması (Climb Limelight ile) ──
        driverController.pov(180)
            .whileTrue(new AutoClimbAlignCommand(drivetrain, vision));

        // ── X Tuşu: Açı - Podium (Uzak Atış) Preset'i ──
        driverController.x()
            .onTrue(Commands.runOnce(
                () -> shooter.setAngle(ShooterConstants.ANGLE_PODIUM), shooter
            ));

        // ── A Tuşu: Açıyı sıfırla/topla (Stow) ──
        driverController.a()
            .onTrue(Commands.runOnce(
                shooter::stowAngle, shooter
            ));

        // ── Y Tuşu: Seviye 3 için Tırmanma ──
        driverController.y()
            .onTrue(Commands.runOnce(
                climb::climbToLevel3, climb
            ));
    }

    // ────────────── OTONOM (AUTONOMOUS) ──────────────

    /**
     * Çalıştırılacak otonom komutunu döndürür.
     * Sezon ilerledikçe burayı PathPlanner veya Choreo rotalarıyla değiştirebilirsiniz.
     */
    public Command getAutonomousCommand() {
        // Geçici Otonom: Sadece 2 saniye ileri sür ve dur.
        return Commands.run(
            () -> drivetrain.drive(1.0, 0, 0, false), drivetrain
        ).withTimeout(2.0)
         .andThen(Commands.runOnce(drivetrain::stop, drivetrain));
    }
}
