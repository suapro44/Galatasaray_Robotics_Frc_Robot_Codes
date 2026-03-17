package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

/**
 * Intake subsystem — two NEO motors (SparkMax) in a leader/follower
 * configuration.  The follower mirrors the leader's output automatically.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax leaderMotor;
    private final SparkMax followerMotor;

    public IntakeSubsystem() {
        leaderMotor = new SparkMax(IntakeConstants.LEADER_ID, MotorType.kBrushless);
        followerMotor = new SparkMax(IntakeConstants.FOLLOWER_ID, MotorType.kBrushless);

        // Leader config
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig
            .smartCurrentLimit(IntakeConstants.CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);
        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follower config — follows leader, same direction
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .smartCurrentLimit(IntakeConstants.CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake)
            .follow(leaderMotor, false);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ────────────── PUBLIC API ──────────────

    /** Run intake forward at the configured speed. */
    public void runIntake() {
        leaderMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    /** Run intake in reverse (eject). */
    public void reverseIntake() {
        leaderMotor.set(-IntakeConstants.INTAKE_SPEED);
    }

    /** Stop the intake motors. */
    public void stop() {
        leaderMotor.set(0.0);
    }

    /** @return Whether the intake is currently running. */
    public boolean isRunning() {
        return Math.abs(leaderMotor.get()) > 0.05;
    }

    // ────────────── PERIODIC ──────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/LeaderOutput", leaderMotor.get());
        SmartDashboard.putNumber("Intake/LeaderCurrent", leaderMotor.getOutputCurrent());
    }
}
