package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ChamberConstants;

/**
 * Chamber (transfer) subsystem — single NEO motor that moves the game piece
 * from the intake through to the shooter.
 */
public class ChamberSubsystem extends SubsystemBase {

    private final SparkMax motor;

    public ChamberSubsystem() {
        motor = new SparkMax(ChamberConstants.MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(ChamberConstants.CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ────────────── PUBLIC API ──────────────

    /** Run the chamber forward at configured speed. */
    public void runChamber() {
        motor.set(ChamberConstants.CHAMBER_SPEED);
    }

    /** Run the chamber in reverse. */
    public void reverseChamber() {
        motor.set(-ChamberConstants.CHAMBER_SPEED);
    }

    /** Stop the chamber motor. */
    public void stop() {
        motor.set(0.0);
    }

    /** @return Whether the chamber is currently moving. */
    public boolean isRunning() {
        return Math.abs(motor.get()) > 0.05;
    }

    // ────────────── PERIODIC ──────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Chamber/Output", motor.get());
        SmartDashboard.putNumber("Chamber/Current", motor.getOutputCurrent());
    }
}
