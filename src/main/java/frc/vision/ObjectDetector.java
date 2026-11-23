
package frc.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.IntakeConstants.ObjectDetectionConstants;

public final class ObjectDetector implements Sendable {
    private static final Comparator<Detection> SORT = Comparator.comparingDouble(Detection::area)
            .thenComparingDouble(Detection::confidence)
            .reversed();

    private final PhotonCamera camera;

    private List<Detection> targets = List.of();
    private double frameTime = Double.NaN;

    public record Detection(
            int classId,
            double confidence,
            double yawDegrees,
            double pitchDegrees,
            double x,
            double y,
            double width,
            double height,
            double orientationDegrees) {

        public double area() {
            return width * height;
        }

        public double left() {
            return x - width / 2.0;
        }

        public double right() {
            return x + width / 2.0;
        }

        public double top() {
            return y - height / 2.0;
        }

        public double bottom() {
            return y + height / 2.0;
        }
    }

    public ObjectDetector() {
        camera = new PhotonCamera(ObjectDetectionConstants.cameraName);
        camera.setDriverMode(false);
        camera.setPipelineIndex(ObjectDetectionConstants.pipelineIndex);
    }

    public void update() {
        var frames = camera.getAllUnreadResults();

        if (!isConnected() || !isCorrectPipeline()) {
            clear();
            return;
        }

        if (!frames.isEmpty()) {
            var frame = frames.get(frames.size() - 1);
            frameTime = frame.getTimestampSeconds();

            var found = new ArrayList<Detection>();

            for (var target : frame.getTargets()) {
                readTarget(target).ifPresent(found::add);
            }

            found.sort(SORT);
            targets = List.copyOf(found);
        }

        if (!hasFreshFrame()) {
            targets = List.of();
        }
    }

    private Optional<Detection> readTarget(PhotonTrackedTarget target) {
        int id = target.getDetectedObjectClassID();
        double confidence = target.getDetectedObjectConfidence();

        if (id < 0
                || !Double.isFinite(confidence)
                || confidence < 0.0
                || confidence > 1.0
                || confidence < ObjectDetectionConstants.minConfidence) {
            return Optional.empty();
        }

        int wanted = ObjectDetectionConstants.targetClassId;

        if (wanted >= 0 && id != wanted) {
            return Optional.empty();
        }

        double yaw = target.getYaw();
        double pitch = target.getPitch();

        if (!Double.isFinite(yaw) || !Double.isFinite(pitch)) {
            return Optional.empty();
        }

        var corners = target.getMinAreaRectCorners();

        if (corners.size() != 4) {
            return Optional.empty();
        }

        double left = Double.POSITIVE_INFINITY;
        double right = Double.NEGATIVE_INFINITY;
        double top = Double.POSITIVE_INFINITY;
        double bottom = Double.NEGATIVE_INFINITY;

        for (var corner : corners) {
            if (!Double.isFinite(corner.x)
                    || !Double.isFinite(corner.y)) {
                return Optional.empty();
            }

            left = Math.min(left, corner.x);
            right = Math.max(right, corner.x);
            top = Math.min(top, corner.y);
            bottom = Math.max(bottom, corner.y);
        }

        double width = right - left;
        double height = bottom - top;
        double area = width * height;

        if (width <= 0.0
                || height <= 0.0
                || !Double.isFinite(area)
                || area < ObjectDetectionConstants.minBoxAreaPixels) {
            return Optional.empty();
        }

        return Optional.of(new Detection(
                id,
                confidence,
                yaw,
                pitch,
                left + width / 2.0,
                top + height / 2.0,
                width,
                height,
                orientationDegrees(corners)));
    }

    /**
     * Orientation of the box's long side in image space, from the minimum-area
     * rectangle corners. Returned in degrees in (-90, 90], measured from the
     * image x-axis (positive is clockwise on screen, since image y points down).
     */
    private static double orientationDegrees(List<TargetCorner> corners) {
        double bestLength = -1.0;
        double bestAngle = 0.0;

        for (int i = 0; i < 4; i++) {
            var a = corners.get(i);
            var b = corners.get((i + 1) % 4);
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            double length = Math.hypot(dx, dy);

            if (length > bestLength) {
                bestLength = length;
                bestAngle = Math.toDegrees(Math.atan2(dy, dx));
            }
        }

        while (bestAngle > 90.0) {
            bestAngle -= 180.0;
        }
        while (bestAngle <= -90.0) {
            bestAngle += 180.0;
        }

        return bestAngle;
    }

    /**
     * Locates the best detected game piece relative to the robot's camera.
     *
     * @param cameraHeightMeters     height of the camera lens above the floor
     * @param cameraAngleDownDegrees how far the camera is tilted down from horizontal
     * @return the piece's position and orientation, or empty if there is no target
     *         or the camera calibration constants are not set
     */
    public Optional<GamePieceLocator.GamePiece> getBestGamePiece(
            double cameraHeightMeters, double cameraAngleDownDegrees) {
        return getBestTarget().flatMap(detection -> GamePieceLocator.locate(
                detection, cameraHeightMeters, cameraAngleDownDegrees));
    }

    private void clear() {
        targets = List.of();
        frameTime = Double.NaN;
    }

    public boolean isConnected() {
        return camera.isConnected();
    }

    public boolean isCorrectPipeline() {
        return !camera.getDriverMode()
                && camera.getPipelineIndex() == ObjectDetectionConstants.pipelineIndex;
    }

    public double getFrameAgeSeconds() {
        if (!Double.isFinite(frameTime)) {
            return Double.POSITIVE_INFINITY;
        }

        double age = Timer.getFPGATimestamp() - frameTime;

        if (!Double.isFinite(age)
                || age < -ObjectDetectionConstants.futureTimestampToleranceSeconds) {
            return Double.POSITIVE_INFINITY;
        }

        return Math.max(0.0, age);
    }

    public boolean hasFreshFrame() {
        return isConnected()
                && isCorrectPipeline()
                && getFrameAgeSeconds() <= ObjectDetectionConstants.staleTimeoutSeconds;
    }

    /**
     * Estimates the horizontal distance from the camera to a detection, in meters.
     *
     * <p>Assumes the object sits on a flat floor at targetHeightMeters. The angle
     * of the ray below horizontal is the camera pitch plus the target pitch (both
     * positive up), so distance = (cameraHeight - targetHeight) / tan(angleDown).
     *
     * @return the distance, or empty if the ray does not point down at the object
     */
    public static Optional<Double> estimateDistanceMeters(Detection detection) {
        double cameraHeight = ObjectDetectionConstants.cameraHeightMeters;
        double targetHeight = ObjectDetectionConstants.targetHeightMeters;
        double angleDownDegrees = -(ObjectDetectionConstants.cameraPitchDegrees + detection.pitchDegrees());

        if (cameraHeight <= targetHeight || angleDownDegrees <= 0.0) {
            return Optional.empty();
        }

        double distance = (cameraHeight - targetHeight) / Math.tan(Math.toRadians(angleDownDegrees));

        if (!Double.isFinite(distance) || distance < 0.0) {
            return Optional.empty();
        }

        return Optional.of(distance);
    }

    public Optional<Double> getBestTargetDistanceMeters() {
        return getBestTarget().flatMap(ObjectDetector::estimateDistanceMeters);
    }

    public List<Detection> getDetections() {
        return hasFreshFrame() ? targets : List.of();
    }

    public List<Detection> getDetections(int id) {
        return getDetections().stream()
                .filter(target -> target.classId() == id)
                .toList();
    }

    public Optional<Detection> getBestTarget() {
        return getDetections().stream().findFirst();
    }

    public Optional<Detection> getBestTarget(int id) {
        return getDetections(id).stream().findFirst();
    }

    public boolean hasTarget() {
        return getBestTarget().isPresent();
    }

    public boolean hasTarget(int id) {
        return getBestTarget(id).isPresent();
    }

    public int getTargetCount() {
        return getDetections().size();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Object Detector");

        builder.addDoubleProperty(
                "Target Distance Meters",
                () -> getBestTargetDistanceMeters().orElse(-1.0),
                null);
    }
}
