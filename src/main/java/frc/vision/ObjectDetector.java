/*
package frc.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Constants.ObjectDetectionConstants;

public final class ObjectDetector implements Sendable {
    private static final Comparator<Detection> SORT =
            Comparator.comparingDouble(Detection::area)
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
            double height) {

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
                height));
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
                && camera.getPipelineIndex()
                        == ObjectDetectionConstants.pipelineIndex;
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
                && getFrameAgeSeconds()
                        <= ObjectDetectionConstants.staleTimeoutSeconds;
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

        builder.addBooleanProperty(
                "Connected", this::isConnected, null);
        builder.addBooleanProperty(
                "Correct Pipeline", this::isCorrectPipeline, null);
        builder.addBooleanProperty(
                "Fresh Frame", this::hasFreshFrame, null);
        builder.addBooleanProperty(
                "Has Target", this::hasTarget, null);

        builder.addDoubleProperty(
                "Target Count", this::getTargetCount, null);

        builder.addDoubleProperty("Frame Age Seconds", () -> {
            double age = getFrameAgeSeconds();
            return Double.isFinite(age) ? age : -1.0;
        }, null);

        builder.addDoubleProperty(
                "Target X",
                () -> getBestTarget().map(Detection::x).orElse(0.0),
                null);
        builder.addDoubleProperty(
                "Target Y",
                () -> getBestTarget().map(Detection::y).orElse(0.0),
                null);
        builder.addDoubleProperty(
                "Target Width",
                () -> getBestTarget().map(Detection::width).orElse(0.0),
                null);
        builder.addDoubleProperty(
                "Target Height",
                () -> getBestTarget().map(Detection::height).orElse(0.0),
                null);
        builder.addDoubleProperty(
                "Target Area Pixels",
                () -> getBestTarget().map(Detection::area).orElse(0.0),
                null);
        builder.addDoubleProperty(
                "Target Yaw Degrees",
                () -> getBestTarget().map(Detection::yawDegrees).orElse(0.0),
                null);
        builder.addDoubleProperty(
                "Target Pitch Degrees",
                () -> getBestTarget().map(Detection::pitchDegrees).orElse(0.0),
                null);
        builder.addDoubleProperty(
                "Target Confidence",
                () -> getBestTarget().map(Detection::confidence).orElse(0.0),
                null);
        builder.addDoubleProperty(
                "Target Class ID",
                () -> getBestTarget()
                        .map(target -> (double) target.classId())
                        .orElse(-1.0),
                null);
    }
}
*/
*/ 
