package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The main robot class. This is a TimedRobot that delegates all subsystem
 * and command configuration to {@link RobotContainer}.
 */
public class Robot extends TimedRobot {

    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        // Instantiate RobotContainer — this performs all subsystem and command setup
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Run the scheduler — this is the heartbeat of the Command-Based framework
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // No-op
    }

    @Override
    public void disabledPeriodic() {
        // No-op
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // No-op — CommandScheduler handles everything
    }

    @Override
    public void teleopInit() {
        // Cancel autonomous command when teleop starts
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        // No-op — CommandScheduler handles everything
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
