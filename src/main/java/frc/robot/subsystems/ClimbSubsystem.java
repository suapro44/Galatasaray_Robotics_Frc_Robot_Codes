package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;

/**
 * Climb subsystem — single NEO motor (SparkMax) with PID position control
 * via the internal relative encoder.  Provides a one-button climb to Level 3.
 */
public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    private double targetPosition = ClimbConstants.STOW_SETPOINT;

    public ClimbSubsystem() {
        motor = new SparkMax(ClimbConstants.MOTOR_ID, MotorType.kBrushless);

        // Configure motor with PID
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(ClimbConstants.CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);
        config.closedLoop
            .p(ClimbConstants.CLIMB_KP)
            .i(ClimbConstants.CLIMB_KI)
            .d(ClimbConstants.CLIMB_KD)
            .outputRange(ClimbConstants.CLIMB_MIN_OUTPUT, ClimbConstants.CLIMB_MAX_OUTPUT);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Relative encoder — zero position at startup (fully retracted)
        encoder = motor.getEncoder();
        encoder.setPosition(0.0);

        // Closed-loop controller
        pidController = motor.getClosedLoopController();
    }

    // ────────────── PUBLIC API ──────────────

    /**
     * Command the climber to Level 3 height.
     */
    public void climbToLevel3() {
        setPosition(ClimbConstants.LEVEL_3_SETPOINT);
    }

    /**
     * Retract the climber to stow position.
     */
    public void stow() {
        setPosition(ClimbConstants.STOW_SETPOINT);
    }

    /**
     * Set the climber to an arbitrary position target.
     *
     * @param position Target in encoder rotations.
     */
    public void setPosition(double position) {
        this.targetPosition = position;
        pidController.setReference(position, ControlType.kPosition);
    }

    /** Stop the climb motor (holds via brake mode). */
    public void stop() {
        motor.set(0.0);
    }

    /**
     * @return Whether the climber is within tolerance of its target position.
     */
    public boolean isAtTarget() {
        return Math.abs(encoder.getPosition() - targetPosition) < ClimbConstants.POSITION_TOLERANCE;
    }

    /** @return Current encoder position (rotations). */
    public double getCurrentPosition() {
        return encoder.getPosition();
    }

    // ────────────── PERIODIC ──────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/Position", encoder.getPosition());
        SmartDashboard.putNumber("Climb/Target", targetPosition);
        SmartDashboard.putBoolean("Climb/AtTarget", isAtTarget());
        SmartDashboard.putNumber("Climb/Current", motor.getOutputCurrent());
    }
}
