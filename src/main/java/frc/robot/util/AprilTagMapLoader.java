package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;

import java.io.IOException;

public class AprilTagMapLoader {
    public static AprilTagFieldLayout tagFieldLayout;

    public static void main(String[] args) {
        try {
            tagFieldLayout = AprilTagFieldLayout
                    .loadFromResource("src/main/deploy/Bunnybots_2026.json");
        } catch (IOException e) {
            System.err.println("Failed to load AprilTag field layout: " + e.getMessage());
        }
    }

    public static AprilTagFieldLayout getTagFieldLayout() {
        return tagFieldLayout;
    }
}