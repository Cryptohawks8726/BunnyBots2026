package frc.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Constants.ObjectDetectionConstants;
import frc.vision.ObjectDetector.Detection;

/**
 * Converts a game piece's bounding box into a position relative to the camera.
 *
 * <p>Uses only the bounding box (its center pixel and its orientation) plus the
 * camera's height, its downward tilt, and its calibration constants. The
 * piece's center is assumed to be at ObjectDetectionConstants.targetHeightMeters.
 *
 * <p>Coordinates follow WPILib conventions: x is forward from the camera, y is
 * left of the camera, and z is up. Angles are in degrees.
 */
public final class GamePieceLocator {

    private GamePieceLocator() {}

    /**
     * A game piece located relative to the camera.
     *
     * @param positionRelativeToCamera forward (x) and left (y) distance from the camera, in meters
     * @param heightMeters             height of the piece's center above the floor, in meters
     * @param orientationDegrees       orientation of the piece's long side in image space
     *                                 (see ObjectDetector.Detection), in degrees
     */
    public record GamePiece(
            Translation2d positionRelativeToCamera,
            double heightMeters,
            double orientationDegrees) {

        /** Straight-line distance from the camera to the piece, in meters. */
        public double distanceMeters() {
            return positionRelativeToCamera.getNorm();
        }
    }

    /**
     * Locates a game piece from its bounding box.
     *
     * @param detection              the detection, using its pixel center and orientation
     * @param cameraHeightMeters     height of the camera lens above the floor, in meters
     * @param cameraAngleDownDegrees tilt of the camera below horizontal, in degrees (positive is down)
     * @return the piece's position relative to the camera, or empty if the ray does
     *         not hit the piece's height in front of the camera, or the camera
     *         calibration constants are not set
     */
    public static Optional<GamePiece> locate(
            Detection detection,
            double cameraHeightMeters,
            double cameraAngleDownDegrees) {
        double width = ObjectDetectionConstants.imageWidthPixels;
        double height = ObjectDetectionConstants.imageHeightPixels;
        double hfov = ObjectDetectionConstants.horizontalFovDegrees;
        double vfov = ObjectDetectionConstants.verticalFovDegrees;

        if (width <= 0.0 || height <= 0.0 || hfov <= 0.0 || vfov <= 0.0 || hfov >= 180.0 || vfov >= 180.0) {
            return Optional.empty();
        }

        // Focal lengths in pixels, from the field of view.
        double fx = (width / 2.0) / Math.tan(Math.toRadians(hfov / 2.0));
        double fy = (height / 2.0) / Math.tan(Math.toRadians(vfov / 2.0));

        // Normalized image coordinates of the box center: right and up are positive.
        double nx = (detection.x() - width / 2.0) / fx;
        double ny = (height / 2.0 - detection.y()) / fy;

        // Ray through the box center, in a level frame (forward, left, up),
        // with the camera tilted down by cameraAngleDownDegrees.
        double angle = Math.toRadians(cameraAngleDownDegrees);
        double rayForward = Math.cos(angle) + ny * Math.sin(angle);
        double rayLeft = -nx;
        double rayUp = -Math.sin(angle) + ny * Math.cos(angle);

        // The ray must point down toward the piece's height.
        if (rayUp >= 0.0) {
            return Optional.empty();
        }

        double targetHeight = ObjectDetectionConstants.targetHeightMeters;
        double t = (targetHeight - cameraHeightMeters) / rayUp;

        if (t <= 0.0) {
            return Optional.empty();
        }

        double forward = t * rayForward;
        double left = t * rayLeft;

        if (!Double.isFinite(forward) || !Double.isFinite(left)) {
            return Optional.empty();
        }

        return Optional.of(new GamePiece(
                new Translation2d(forward, left),
                targetHeight,
                detection.orientationDegrees()));
    }
}
