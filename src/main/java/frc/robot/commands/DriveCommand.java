package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Default drive command that continuously reads joystick inputs and
 * controls the swerve drivetrain in field-centric mode.
 */
public class DriveCommand extends Command {

    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier rotationSupplier;

    /**
     * @param drivetrain       The drivetrain subsystem.
     * @param xSpeedSupplier   Forward/backward axis (positive = forward).
     * @param ySpeedSupplier   Left/right axis (positive = left).
     * @param rotationSupplier Rotation axis (positive = counter-clockwise).
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
        // Apply deadband and scale to max speed
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
        return false; // Runs until interrupted
    }
}
