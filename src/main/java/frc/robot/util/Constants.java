// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static final boolean demoMode = false;
        // CHANGE BEFORE MATCHES!!!!!
        public static final Alliance currentAlliance = Alliance.Blue;

        public static final AprilTagFieldLayout aprilTagLayout = AprilTagMapLoader.getTagFieldLayout();

        public static final class SwerveConstants {
                public static final double maxSpeed = (demoMode ? 2.0 : 4.8); // m/s
                public static final double maxAngularSpeed = (demoMode ? 1.5 : 3); // rad/s

                public static final double defaultTranslationCoefficient = 1;
                public static final double defaultRotationCoefficient = 1;
                public static final double slowTranslationCoefficient = 0.3;
                public static final double slowRotationCoefficient = 0.3;

                public static final double positionkP = 0.5;
                public static final double positionkI = 0.0;
                public static final double positionkD = 0.0;

                public static final double rotationkP = 0.5;
                public static final double rotationkI = 0.0;
                public static final double rotationkD = 0.0;

                public static final double translationalErrorRange = 0.02;
                public static final double rotationalErrorRange = 0.1; // Radians

                public static final double joystickDeadband = 0.1; // TODO: UPDATE this if drivers want
        }

        public static final class OdometryConstants {
                // rad/s
                public static final double ODO_TRUST_ROTATIONAL_SPEED_LIMIT = 2 * Math.PI;

                public static final Matrix<N3, N1> REGULAR = new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {
                                0.1, 0.1, 0.1
                });

                public static final Matrix<N3, N1> SLIGHTLY_UNTRUSTED = new Matrix<N3, N1>(Nat.N3(), Nat.N1(),
                                new double[] {
                                                0.2, 0.2, 0.2
                                });

                public static final Matrix<N3, N1> SOMETHING_ELSE_IS_100 = new Matrix<N3, N1>(Nat.N3(), Nat.N1(),
                                new double[] {
                                                0.30, 0.30, 0.30
                                });

                public static final Matrix<N3, N1> FULLY_TRUSTED = new Matrix<N3, N1>(Nat.N3(), Nat.N1(),
                                new double[] { 0.01, 0.01, 0.01 });
                // FOR LIMELIGHTs and PhotonVision Cams
                // x axis is forward bot distance from tag
                // y axis is offset right (positive) or left
                // rotation is measured in radians
                // distance is measured in meters
                // TODO: UPDATE DURING BOT FABRICATION
                // Camera position relative to robot center
                // LL Cam
                public static final double ll1forwardOffset = 0; // 0 in
                public static final double ll1sideOffset = 0; // 0 in
                public static final double ll1upOffset = 0; // 0 in
                public static final double ll1rollOffset = 0; // we will most likely never use roll
                public static final double ll1pitchOffset = 20; // Angled up or down
                public static final double ll1yawOffset = 0; // Angled left or right
                // PV Cam
                public static final Transform3d pv1Offset = new Transform3d(
                                0, // 0 in (forward offset)
                                0, // 0 in (right offset)
                                0, // 0in (up offset)
                                new Rotation3d(
                                                0, // roll (most likely never used)
                                                0, // pitch (up or down)
                                                0 // yaw (left or right)
                                ));

                public static final Pose2d ovenScoringPos = currentAlliance == Alliance.Blue ? new Pose2d()
                                : new Pose2d();
        }

        public static final class IndexerConstants { // TODO: CHANGE ALL IDS

                // sparkmax roller motor ids
                public static final int shooterMotorID = 50;
                public static final int indexerMotorOneID = 51;
                public static final int indexerMotorTwoID = 52;
                public static final int indexerMotorThreeID = 53;

                // lidar ids
                public static final int lidarOneID = 5; // change all ids later
                public static final int lidarTwoID = 6;
                public static final int lidarThreeID = 7;

                public static final double rollerTargetRPM = 5.0;
                public static final double rollerVoltage = 4;

                public static final double lidarDetection = 0;
        }

        // CHANGE THESE DURING TESING!
        public static final class IntakeConstants {
                // Motors
                public static final int armMotorID = 54;
                public static final int rollerMotorID = 55;

                // PID
                public static final double arm_kP = 1.0;
                public static final double arm_kI = 0.0;
                public static final double arm_kD = 0.0;

                // Arm
                public static final double armRaisedPosition = 0.0;
                public static final double armPartialRaisedPosition = 0.0;
                public static final double armLoweredPosition = 0.0;

                public static final double armConversionFactor = 0.0;
                public static final double armTolerance = 0.0;
                public static final double arm_kG = 0.0;
                public static final double arm_kS = 0.0;

                // Roller
                public static final double desiredRollerRPM = 0.0;
                public static final double acceptedRollerError = 0.0;
                public static final double rollerVelocityConversionFactor = 0.0;
                public static final double rollerVoltage = 1;

        }

        public static final class ObjectDetectionConstants {
                public static final String cameraName = "placeholder";
                public static final int pipelineIndex = 0;

                public static final double minConfidence = 0.5;
                public static final int targetClassId = -1;
                public static final double minBoxAreaPixels = 0.0;

                public static final double futureTimestampToleranceSeconds = 0.05;
                public static final double staleTimeoutSeconds = 0.5;

                // Camera mounting, measured on the robot. TODO: measure these.
                // Height is from the floor to the camera lens, in meters.
                public static final double cameraHeightMeters = 0.0;
                // Pitch is the camera tilt from horizontal, in degrees. Positive = tilted up.
                public static final double cameraPitchDegrees = 0.0;
                // Height of the center of the object above the floor, in meters.
                public static final double targetHeightMeters = 0.0;

                // Camera calibration, from the camera's stream resolution and field of view.
                // TODO: calibrate these. Until they are set, locating game pieces returns empty.
                public static final double imageWidthPixels = 0.0;
                public static final double imageHeightPixels = 0.0;
                public static final double horizontalFovDegrees = 0.0;
                public static final double verticalFovDegrees = 0.0;
        }

}
