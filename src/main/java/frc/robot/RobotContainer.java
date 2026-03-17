package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ChamberSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to commands and command groups. It is also where subsystems
 * are instantiated.
 *
 * <h2>Controller Layout (Xbox)</h2>
 * <table>
 *   <tr><td>Left Stick</td><td>Field-centric translation (X/Y)</td></tr>
 *   <tr><td>Right Stick X</td><td>Rotation</td></tr>
 *   <tr><td>Start Button</td><td>Zero gyro heading</td></tr>
 *   <tr><td>Right Bumper</td><td>Intake (hold)</td></tr>
 *   <tr><td>Left Bumper</td><td>Chamber feed (hold)</td></tr>
 *   <tr><td>Right Trigger</td><td>Shoot (spin up + feed)</td></tr>
 *   <tr><td>Left Trigger</td><td>Angle preset (Subwoofer)</td></tr>
 *   <tr><td>A Button</td><td>Stow shooter angle</td></tr>
 *   <tr><td>B Button</td><td>Reverse intake (eject)</td></tr>
 *   <tr><td>X Button</td><td>Angle preset (Podium)</td></tr>
 *   <tr><td>Y Button</td><td>Climb to Level 3</td></tr>
 * </table>
 */
public class RobotContainer {

    // ── Subsystems ──
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final IntakeSubsystem     intake     = new IntakeSubsystem();
    private final ChamberSubsystem    chamber    = new ChamberSubsystem();
    private final ShooterSubsystem    shooter    = new ShooterSubsystem();
    private final ClimbSubsystem      climb      = new ClimbSubsystem();
    private final VisionSubsystem     vision     = new VisionSubsystem();

    // ── Controllers ──
    private final CommandXboxController driverController =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    // ────────────── DEFAULT COMMANDS ──────────────

    private void configureDefaultCommands() {
        // Default drive command — left stick for translation, right stick X for rotation
        // Note: Y axis is negated because pushing the stick forward gives a negative value
        drivetrain.setDefaultCommand(
            new DriveCommand(
                drivetrain,
                () -> -driverController.getLeftY(),   // forward/backward
                () -> -driverController.getLeftX(),   // left/right strafe
                () -> -driverController.getRightX()   // rotation
            )
        );
    }

    // ────────────── BUTTON BINDINGS ──────────────

    private void configureButtonBindings() {

        // ── Start: Zero Gyro ──
        driverController.start().onTrue(
            Commands.runOnce(() -> drivetrain.zeroHeading(), drivetrain)
        );

        // ── Right Bumper: Intake (hold to run, release to stop) ──
        driverController.rightBumper()
            .whileTrue(Commands.startEnd(
                intake::runIntake,
                intake::stop,
                intake
            ));

        // ── B Button: Reverse Intake / Eject (hold) ──
        driverController.b()
            .whileTrue(Commands.startEnd(
                intake::reverseIntake,
                intake::stop,
                intake
            ));

        // ── Left Bumper: Chamber Feed (hold to run) ──
        driverController.leftBumper()
            .whileTrue(Commands.startEnd(
                chamber::runChamber,
                chamber::stop,
                chamber
            ));

        // ── Right Trigger (> 0.5): Shoot sequence ──
        //    1. Spin up shooter wheels
        //    2. Wait until angle is on target
        //    3. Feed chamber
        //    Release to stop everything
        driverController.rightTrigger(0.5)
            .whileTrue(
                Commands.parallel(
                    // Spin up shooter
                    Commands.startEnd(
                        shooter::runShooter,
                        shooter::stopShooter,
                        shooter
                    ),
                    // Feed chamber after a short delay for spin-up
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

        // ── Left Trigger (> 0.5): Angle to Subwoofer preset ──
        driverController.leftTrigger(0.5)
            .onTrue(Commands.runOnce(
                () -> shooter.setAngle(ShooterConstants.ANGLE_SUBWOOFER), shooter
            ));

        // ── X Button: Angle to Podium preset ──
        driverController.x()
            .onTrue(Commands.runOnce(
                () -> shooter.setAngle(ShooterConstants.ANGLE_PODIUM), shooter
            ));

        // ── A Button: Stow shooter angle ──
        driverController.a()
            .onTrue(Commands.runOnce(
                shooter::stowAngle, shooter
            ));

        // ── Y Button: Climb to Level 3 ──
        driverController.y()
            .onTrue(Commands.runOnce(
                climb::climbToLevel3, climb
            ));
    }

    // ────────────── AUTONOMOUS ──────────────

    /**
     * Returns the autonomous command to run.  Replace with your path-following
     * or auto routine as the season progresses.
     */
    public Command getAutonomousCommand() {
        // Placeholder — drive forward for 2 seconds then stop
        return Commands.run(
            () -> drivetrain.drive(1.0, 0, 0, false), drivetrain
        ).withTimeout(2.0)
         .andThen(Commands.runOnce(drivetrain::stop, drivetrain));
    }
}
