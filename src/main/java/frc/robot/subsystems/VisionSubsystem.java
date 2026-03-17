package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;

/**
 * Vision subsystem interfacing with two Limelight cameras via NetworkTables.
 * <ul>
 *   <li><b>Main Limelight (3)</b>  — AprilTag & game piece (Fuel) detection</li>
 *   <li><b>Climb Limelight (3A)</b> — AprilTag detection for climbing alignment</li>
 * </ul>
 *
 * Provides getter methods so other subsystems / commands can query alignment data.
 */
public class VisionSubsystem extends SubsystemBase {

    // ── NetworkTable entries — Main Limelight ──
    private final NetworkTableEntry mainTx;
    private final NetworkTableEntry mainTy;
    private final NetworkTableEntry mainTv;

    // ── NetworkTable entries — Climb Limelight ──
    private final NetworkTableEntry climbTx;
    private final NetworkTableEntry climbTy;
    private final NetworkTableEntry climbTv;

    public VisionSubsystem() {
        NetworkTable mainTable  = NetworkTableInstance.getDefault()
                .getTable(VisionConstants.MAIN_LIMELIGHT);
        NetworkTable climbTable = NetworkTableInstance.getDefault()
                .getTable(VisionConstants.CLIMB_LIMELIGHT);

        mainTx = mainTable.getEntry("tx");
        mainTy = mainTable.getEntry("ty");
        mainTv = mainTable.getEntry("tv");

        climbTx = climbTable.getEntry("tx");
        climbTy = climbTable.getEntry("ty");
        climbTv = climbTable.getEntry("tv");
    }

    // ────────────── MAIN LIMELIGHT GETTERS ──────────────

    /** @return Horizontal offset from crosshair to target (degrees). */
    public double getMainTx() {
        return mainTx.getDouble(0.0);
    }

    /** @return Vertical offset from crosshair to target (degrees). */
    public double getMainTy() {
        return mainTy.getDouble(0.0);
    }

    /** @return Whether the main Limelight has a valid target (1.0 = yes). */
    public boolean hasMainTarget() {
        return mainTv.getDouble(0.0) >= 1.0;
    }

    // ────────────── CLIMB LIMELIGHT GETTERS ──────────────

    /** @return Horizontal offset from crosshair to target (degrees). */
    public double getClimbTx() {
        return climbTx.getDouble(0.0);
    }

    /** @return Vertical offset from crosshair to target (degrees). */
    public double getClimbTy() {
        return climbTy.getDouble(0.0);
    }

    /** @return Whether the climb Limelight has a valid target. */
    public boolean hasClimbTarget() {
        return climbTv.getDouble(0.0) >= 1.0;
    }

    // ────────────── PIPELINE CONTROL ──────────────

    /**
     * Set the vision processing pipeline on the main Limelight.
     *
     * @param pipeline Pipeline index (0–9).
     */
    public void setMainPipeline(int pipeline) {
        NetworkTableInstance.getDefault()
            .getTable(VisionConstants.MAIN_LIMELIGHT)
            .getEntry("pipeline")
            .setNumber(pipeline);
    }

    /**
     * Set the vision processing pipeline on the climb Limelight.
     *
     * @param pipeline Pipeline index (0–9).
     */
    public void setClimbPipeline(int pipeline) {
        NetworkTableInstance.getDefault()
            .getTable(VisionConstants.CLIMB_LIMELIGHT)
            .getEntry("pipeline")
            .setNumber(pipeline);
    }

    // ────────────── PERIODIC ──────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vision/Main/tx", getMainTx());
        SmartDashboard.putNumber("Vision/Main/ty", getMainTy());
        SmartDashboard.putBoolean("Vision/Main/HasTarget", hasMainTarget());

        SmartDashboard.putNumber("Vision/Climb/tx", getClimbTx());
        SmartDashboard.putNumber("Vision/Climb/ty", getClimbTy());
        SmartDashboard.putBoolean("Vision/Climb/HasTarget", hasClimbTarget());
    }
}
