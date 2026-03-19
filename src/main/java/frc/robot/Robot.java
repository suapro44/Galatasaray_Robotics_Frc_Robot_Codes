package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Ana robot sınıfı. Bu bir TimedRobot'tur ve tüm alt sistem 
 * ile komut konfigürasyonlarını {@link RobotContainer} sınıfına devreder.
 */
public class Robot extends TimedRobot {

    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        // RobotContainer'ı oluştur — tüm alt sistemler ve komutlar burada hazırlanır
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Scheduler'ı çalıştır — bu, Komut Tabanlı (Command-Based) mimarinin kalbidir
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // Robot devre dışı (disabled) bırakıldığında çalışır
    }

    @Override
    public void disabledPeriodic() {
        // Robot devre dışı iken sürekli çalışır
    }

    @Override
    public void autonomousInit() {
        // Otonom periyodu başladığında çalışacak komutu al
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // Otonom sırasında sürekli çalışır (Scheduler işlemi devralır)
    }

    @Override
    public void teleopInit() {
        // Teleop (Sürücü kontrolü) başladığında otonom komutunu iptal et
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        // Teleop sırasında sürekli çalışır (Scheduler işlemi devralır)
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        // No-op
    }

    @Override
    public void simulationInit() {
        // No-op
    }

    @Override
    public void simulationPeriodic() {
        // No-op
    }
}
