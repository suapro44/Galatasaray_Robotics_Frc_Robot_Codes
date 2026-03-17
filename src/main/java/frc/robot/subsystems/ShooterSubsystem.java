package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

/**
 * Shooter subsystem containing:
 * <ul>
 *   <li>3× Vortex (SparkFlex) — flywheel wheels (1 leader + 2 followers)</li>
 *   <li>2× NEO 550 (SparkMax) — angle adjustment (1 leader + 1 follower)</li>
 * </ul>
 *
 * The angle is controlled via the NEO 550 internal (relative) encoder
 * and SparkClosedLoopController in position mode.  The mechanism starts at its
 * lowest position (0) at match start.
 */
public class ShooterSubsystem extends SubsystemBase {

    // ── Flywheel motors ──
    private final SparkFlex wheelLeader;
    private final SparkFlex wheelFollower1;
    private final SparkFlex wheelFollower2;

    // ── Angle motors ──
    private final SparkMax angleLeader;
    private final SparkMax angleFollower;

    // ── Angle control ──
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;

    private double angleSetpoint = ShooterConstants.ANGLE_STOW;

    public ShooterSubsystem() {
        // ─────── Flywheel setup ───────
        wheelLeader    = new SparkFlex(ShooterConstants.WHEEL_LEADER_ID,    MotorType.kBrushless);
        wheelFollower1 = new SparkFlex(ShooterConstants.WHEEL_FOLLOWER1_ID, MotorType.kBrushless);
        wheelFollower2 = new SparkFlex(ShooterConstants.WHEEL_FOLLOWER2_ID, MotorType.kBrushless);

        // Leader config
        SparkFlexConfig wheelLeaderConfig = new SparkFlexConfig();
        wheelLeaderConfig
            .smartCurrentLimit(ShooterConstants.WHEEL_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast);
        wheelLeader.configure(wheelLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follower 1 config
        SparkFlexConfig follower1Config = new SparkFlexConfig();
        follower1Config
            .smartCurrentLimit(ShooterConstants.WHEEL_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast)
            .follow(wheelLeader, false);
        wheelFollower1.configure(follower1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follower 2 config
        SparkFlexConfig follower2Config = new SparkFlexConfig();
        follower2Config
            .smartCurrentLimit(ShooterConstants.WHEEL_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast)
            .follow(wheelLeader, false);
        wheelFollower2.configure(follower2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ─────── Angle setup ───────
        angleLeader   = new SparkMax(ShooterConstants.ANGLE_LEADER_ID,   MotorType.kBrushless);
        angleFollower = new SparkMax(ShooterConstants.ANGLE_FOLLOWER_ID, MotorType.kBrushless);

        // Angle leader config with PID
        SparkMaxConfig angleLeaderConfig = new SparkMaxConfig();
        angleLeaderConfig
            .smartCurrentLimit(ShooterConstants.ANGLE_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);
        angleLeaderConfig.closedLoop
            .p(ShooterConstants.ANGLE_KP)
            .i(ShooterConstants.ANGLE_KI)
            .d(ShooterConstants.ANGLE_KD)
            .outputRange(ShooterConstants.ANGLE_MIN_OUTPUT, ShooterConstants.ANGLE_MAX_OUTPUT);
        angleLeader.configure(angleLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Angle follower config
        SparkMaxConfig angleFollowerConfig = new SparkMaxConfig();
        angleFollowerConfig
            .smartCurrentLimit(ShooterConstants.ANGLE_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake)
            .follow(angleLeader, false);
        angleFollower.configure(angleFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Relative encoder — zero position assumed at power-on (lowest position)
        angleEncoder = angleLeader.getEncoder();
        angleEncoder.setPosition(0.0); // reset to zero at startup

        // Closed-loop controller
        anglePID = angleLeader.getClosedLoopController();
    }

    // ────────────── FLYWHEEL API ──────────────

    /**
     * Set the shooter wheel speed (percent output 0–1).
     *
     * @param speed Percent output (0.0 = stopped, 1.0 = full forward).
     */
    public void setShooterSpeed(double speed) {
        wheelLeader.set(speed);
    }

    /** Run the shooter at the default speed defined in constants. */
    public void runShooter() {
        setShooterSpeed(ShooterConstants.DEFAULT_SHOOTER_SPEED);
    }

    /** Stop the flywheel motors. */
    public void stopShooter() {
        wheelLeader.set(0.0);
    }

    // ────────────── ANGLE API ──────────────

    /**
     * Command the shooter angle to a given setpoint (encoder rotations from zero).
     *
     * @param setpoint Target position in encoder rotations.
     */
    public void setAngle(double setpoint) {
        this.angleSetpoint = setpoint;
        anglePID.setReference(setpoint, ControlType.kPosition);
    }

    /** Move to the stowed (lowest) angle. */
    public void stowAngle() {
        setAngle(ShooterConstants.ANGLE_STOW);
    }

    /** Stop the angle motors (holds via brake mode). */
    public void stopAngle() {
        angleLeader.set(0.0);
    }

    /**
     * @return Whether the angle mechanism is within tolerance of the target.
     */
    public boolean isAtTargetAngle() {
        return Math.abs(angleEncoder.getPosition() - angleSetpoint) < ShooterConstants.ANGLE_TOLERANCE;
    }

    /** @return Current angle encoder position (rotations). */
    public double getCurrentAngle() {
        return angleEncoder.getPosition();
    }

    // ────────────── PERIODIC ──────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/WheelOutput", wheelLeader.get());
        SmartDashboard.putNumber("Shooter/WheelCurrent", wheelLeader.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/AnglePosition", angleEncoder.getPosition());
        SmartDashboard.putNumber("Shooter/AngleSetpoint", angleSetpoint);
        SmartDashboard.putBoolean("Shooter/AtTargetAngle", isAtTargetAngle());
    }
}
