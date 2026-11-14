package frc.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Constants.ObjectDetectionConstants;
import frc.robot.util.stateStuff.StatefulSubsystem;

public class ObjectDetector extends StatefulSubsystem {
    private final PhotonCamera camera;

    private List<Detection> detections = List.of();
    private double frameTimestamp = Double.NaN;

    public record Detection(
            int classId,
            double confidence,
            double x,
            double y,
            double width,
            double height) {

        public double area() {
            return width * height;
        }

        public double left() {
            return x - width / 2;
        }

        public double right() {
            return x + width / 2;
        }

        public double top() {
            return y - height / 2;
        }

        public double bottom() {
            return y + height / 2;
        }
    }

    public ObjectDetector() {
        super("Object Detector");

        camera = new PhotonCamera(ObjectDetectionConstants.cameraName);
        camera.setDriverMode(false);
        camera.setPipelineIndex(ObjectDetectionConstants.pipelineIndex);

        putOnDashboard();
    }

    @Override
    public void periodic() {
        var results = camera.getAllUnreadResults();

        if (!isConnected() || !isCorrectPipeline()) {
            clearFrame();
        } else if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            frameTimestamp = result.getTimestampSeconds();

            var accepted = new ArrayList<Detection>();

            for (var target : result.getTargets()) {
                toDetection(target).ifPresent(accepted::add);
            }

            accepted.sort(
                    Comparator.comparingDouble(Detection::area)
                            .thenComparingDouble(Detection::confidence)
                            .reversed());

            detections = List.copyOf(accepted);
        }

        if (!hasFreshFrame()) {
            detections = List.of();
        }

        super.periodic();
    }

    private Optional<Detection> toDetection(PhotonTrackedTarget target) {
        int classId = target.getDetectedObjectClassID();
        double confidence = target.getDetectedObjectConfidence();

        if (classId < 0
                || !Double.isFinite(confidence)
                || confidence < ObjectDetectionConstants.minConfidence
                || confidence > 1) {
            return Optional.empty();
        }

        int wantedClass = ObjectDetectionConstants.targetClassId;

        if (wantedClass >= 0 && classId != wantedClass) {
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
            if (!Double.isFinite(corner.x) || !Double.isFinite(corner.y)) {
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

        if (width <= 0
                || height <= 0
                || !Double.isFinite(area)
                || area < ObjectDetectionConstants.minBoxAreaPixels) {
            return Optional.empty();
        }

        return Optional.of(new Detection(
                classId,
                confidence,
                left + width / 2,
                top + height / 2,
                width,
                height));
    }

    private void clearFrame() {
        detections = List.of();
        frameTimestamp = Double.NaN;
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
        if (!Double.isFinite(frameTimestamp)) {
            return Double.POSITIVE_INFINITY;
        }

        double age = Timer.getFPGATimestamp() - frameTimestamp;

        if (!Double.isFinite(age)
                || age < -ObjectDetectionConstants.futureTimestampToleranceSeconds) {
            return Double.POSITIVE_INFINITY;
        }

        return Math.max(0, age);
    }

    public boolean hasFreshFrame() {
        return isConnected()
                && isCorrectPipeline()
                && getFrameAgeSeconds()
                        <= ObjectDetectionConstants.staleTimeoutSeconds;
    }

    public List<Detection> getDetections() {
        return hasFreshFrame() ? detections : List.of();
    }

    public Optional<Detection> getBestTarget() {
        return getDetections().stream().findFirst();
    }

    public boolean hasTarget() {
        return getBestTarget().isPresent();
    }

    public int getTargetCount() {
        return getDetections().size();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty("Connected", this::isConnected, null);
        builder.addBooleanProperty("Correct Pipeline", this::isCorrectPipeline, null);
        builder.addBooleanProperty("Fresh Frame", this::hasFreshFrame, null);
        builder.addBooleanProperty("Has Target", this::hasTarget, null);

        builder.addDoubleProperty("Target Count",
                () -> getTargetCount(), null);

        builder.addDoubleProperty("Frame Age Seconds", () -> {
            double age = getFrameAgeSeconds();
            return Double.isFinite(age) ? age : -1;
        }, null);

        builder.addDoubleProperty("Target X",
                () -> getBestTarget().map(Detection::x).orElse(0.0), null);

        builder.addDoubleProperty("Target Y",
                () -> getBestTarget().map(Detection::y).orElse(0.0), null);

        builder.addDoubleProperty("Target Width",
                () -> getBestTarget().map(Detection::width).orElse(0.0), null);

        builder.addDoubleProperty("Target Height",
                () -> getBestTarget().map(Detection::height).orElse(0.0), null);

        builder.addDoubleProperty("Target Area Pixels",
                () -> getBestTarget().map(Detection::area).orElse(0.0), null);

        builder.addDoubleProperty("Target Confidence",
                () -> getBestTarget().map(Detection::confidence).orElse(0.0), null);

        builder.addDoubleProperty("Target Class ID",
                () -> getBestTarget()
                        .map(target -> (double) target.classId())
                        .orElse(-1.0),
                null);
    }
}
