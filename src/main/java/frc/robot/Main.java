package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Standard WPILib entry point. Do NOT modify this file
 * unless you fully understand the implications.
 */
public final class Main {

    private Main() {
        // Utility class — prevent instantiation
    }

    public static void main(String[] args) {
        RobotBase.startRobot(Robot::new);
    }
}
