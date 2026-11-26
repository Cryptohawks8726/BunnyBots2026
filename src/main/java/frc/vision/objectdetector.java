package frc.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleFunction;
import java.util.function.ToIntFunction;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Constants.ObjectDetectionConstants;

/**
 * Reads blob detections from a PhotonVision camera and locates each one on the
 * field.
 *
 * The object is assumed to sit on the floor with a roughly round footprint of
 * known radius (ObjectDetectionConstants.objectRadiusMeters). The bottom-center
 * of its box is taken as the floor contact point, a ray is cast from the camera
 * through that pixel, and the ray is intersected with the floor. The result is
 * pushed back by the radius to land on the object's center.
 *
 * The robot pose is needed to convert to field coordinates. It is looked up
 * at the frame's capture time, so camera latency is accounted for. Pass a
 * function that returns the pose estimator's pose at a given timestamp.
 *
 * Call update() once per robot loop, then use the getters.
 */
public final class ObjectDetector implements Sendable {

    private final PhotonCamera camera;
    private final ToIntFunction<PhotonTrackedTarget> classifier;

    // Written by the robot thread in update(), read by the dashboard thread.
    // Lists are replaced, never edited, so they are safe to hand out.
    private volatile List<Detection> detections = List.of();
    private volatile List<LocatedObject> objects = List.of();
    private volatile double frameTime = Double.NaN;
    private volatile boolean cameraOk = false;

    //
    // Data classes
    //

    /**
     * One detected blob, in image pixels with the origin at the top-left and y
     * pointing down. Yaw is positive left and pitch is positive up, following
     * the PhotonVision convention.
     */
    public static final class Detection {
        private final int classId;
        private final double yawDegrees;
        private final double pitchDegrees;
        private final double centerX;
        private final double centerY;
        private final double width;
        private final double height;
        private final double imageAngleDegrees;
        private final List<TargetCorner> corners;
        private final boolean clipped;

        public Detection(
                int classId,
                double yawDegrees,
                double pitchDegrees,
                double centerX,
                double centerY,
                double width,
                double height,
                double imageAngleDegrees,
                List<TargetCorner> corners,
                boolean clipped) {
            this.classId = classId;
            this.yawDegrees = yawDegrees;
            this.pitchDegrees = pitchDegrees;
            this.centerX = centerX;
            this.centerY = centerY;
            this.width = width;
            this.height = height;
            this.imageAngleDegrees = imageAngleDegrees;
            this.corners = List.copyOf(corners);
            this.clipped = clipped;
        }

        public int getClassId() {
            return classId;
        }

        public double getYawDegrees() {
            return yawDegrees;
        }

        public double getPitchDegrees() {
            return pitchDegrees;
        }

        public double getCenterX() {
            return centerX;
        }

        public double getCenterY() {
            return centerY;
        }

        public double getWidth() {
            return width;
        }

        public double getHeight() {
            return height;
        }

        public double getArea() {
            return width * height;
        }

        /** Width divided by height of the box. */
        public double getAspectRatio() {
            return width / height;
        }

        /** Image y of the bottom edge of the box. Bigger means lower in the image. */
        public double getBottomY() {
            return centerY + height / 2.0;
        }

        /**
         * Angle of the box's longest side in image space, from -90 (exclusive) to
         * 90 (inclusive) degrees, measured from the image x-axis (y down).
         */
        public double getImageAngleDegrees() {
            return imageAngleDegrees;
        }

        /** The four corners of the minimum-area rectangle, in image pixels. */
        public List<TargetCorner> getCorners() {
            return corners;
        }

        /**
         * True if the box touches the left, right or bottom image edge, so the
         * floor contact point is cut off and cannot be trusted.
         */
        public boolean isClipped() {
            return clipped;
        }
    }

    /** An object located on the field. */
    public static final class LocatedObject {
        private final Translation2d fieldPosition;
        private final double distanceFromCameraMeters;
        private final int classId;
        private final double aspectRatio;
        private final double imageAngleDegrees;
        private final double frameTimeSeconds;

        public LocatedObject(
                Translation2d fieldPosition,
                double distanceFromCameraMeters,
                int classId,
                double aspectRatio,
                double imageAngleDegrees,
                double frameTimeSeconds) {
            this.fieldPosition = fieldPosition;
            this.distanceFromCameraMeters = distanceFromCameraMeters;
            this.classId = classId;
            this.aspectRatio = aspectRatio;
            this.imageAngleDegrees = imageAngleDegrees;
            this.frameTimeSeconds = frameTimeSeconds;
        }

        /** Center of the object on the field, in meters, field-relative. */
        public Translation2d getFieldPosition() {
            return fieldPosition;
        }

        /** Floor distance from the camera to the object's center, in meters. */
        public double getDistanceFromCameraMeters() {
            return distanceFromCameraMeters;
        }

        public int getClassId() {
            return classId;
        }

        public double getAspectRatio() {
            return aspectRatio;
        }

        public double getImageAngleDegrees() {
            return imageAngleDegrees;
        }

        /** Capture time of the frame in FPGA seconds. */
        public double getFrameTimeSeconds() {
            return frameTimeSeconds;
        }
    }

    // Closest detection first, measured by the lowest point in the image
    private static final Comparator<Detection> LOWEST_FIRST =
            Comparator.comparingDouble(Detection::getBottomY).reversed();

    // Closest object first
    private static final Comparator<LocatedObject> CLOSEST_FIRST =
            Comparator.comparingDouble(LocatedObject::getDistanceFromCameraMeters);

    //
    // Setup and updating
    //

    /** Classifies everything as class 0. Pass your own classifier to tell pieces apart. */
    public ObjectDetector() {
        this(target -> 0);
    }

    /**
     * @param classifier maps a PhotonVision target to a class id, 0 or higher.
     *                   Targets mapped to a negative id are dropped.
     */
    public ObjectDetector(ToIntFunction<PhotonTrackedTarget> classifier) {
        this.classifier = classifier;
        camera = new PhotonCamera(ObjectDetectionConstants.cameraName);
        camera.setDriverMode(false);
        camera.setPipelineIndex(ObjectDetectionConstants.pipelineIndex);
    }

    /**
     * Reads new camera data. Call once per robot loop.
     *
     * @param poseAtTime returns the robot's field pose at a FPGA timestamp, or
     *                   empty if it is not known. For example,
     *                   timestamp -&gt; poseEstimator.sampleAt(timestamp)
     */
    public void update(DoubleFunction<Optional<Pose2d>> poseAtTime) {
        // These are NetworkTables reads, so do them once per loop and cache
        cameraOk = isConnected() && isCorrectPipeline();

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (!cameraOk) {
            clear();
            return;
        }

        // Only the newest result matters
        if (!results.isEmpty()) {
            loadResult(results.get(results.size() - 1), poseAtTime);
        }

        if (getFrameAgeSeconds() > ObjectDetectionConstants.staleTimeoutSeconds) {
            detections = List.of();
            objects = List.of();
        }
    }

    private void loadResult(
            PhotonPipelineResult result, DoubleFunction<Optional<Pose2d>> poseAtTime) {
        double timestamp = result.getTimestampSeconds();

        List<Detection> found = new ArrayList<>();
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                Detection detection = convertTarget(target);
                if (detection != null) {
                    found.add(detection);
                }
            }
        }
        found.sort(LOWEST_FIRST);

        List<LocatedObject> located = new ArrayList<>();
        Optional<Pose2d> robotPose = poseAtTime.apply(timestamp);
        if (robotPose.isPresent()) {
            for (Detection detection : found) {
                locateObject(detection, robotPose.get(), timestamp).ifPresent(located::add);
            }
        }
        located.sort(CLOSEST_FIRST);

        // Publish the lists before the time, so a reader never sees a new time
        // with old lists
        detections = List.copyOf(found);
        objects = List.copyOf(located);
        frameTime = timestamp;
    }

    private void clear() {
        detections = List.of();
        objects = List.of();
        frameTime = Double.NaN;
    }

    //
    // Turning a PhotonVision target into a Detection
    //

    /** Returns the Detection, or null if the target should be ignored. */
    private Detection convertTarget(PhotonTrackedTarget target) {
        int classId = classifier.applyAsInt(target);
        if (classId < 0) {
            return null;
        }

        double yaw = target.getYaw();
        double pitch = target.getPitch();
        if (!Double.isFinite(yaw) || !Double.isFinite(pitch)) {
            return null;
        }

        List<TargetCorner> corners = target.getMinAreaRectCorners();
        if (corners.size() != 4) {
            return null;
        }

        double left = Double.POSITIVE_INFINITY;
        double right = Double.NEGATIVE_INFINITY;
        double top = Double.POSITIVE_INFINITY;
        double bottom = Double.NEGATIVE_INFINITY;

        for (TargetCorner corner : corners) {
            if (!Double.isFinite(corner.x) || !Double.isFinite(corner.y)) {
                return null;
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
            return null;
        }

        return new Detection(
                classId,
                yaw,
                pitch,
                left + width / 2.0,
                top + height / 2.0,
                width,
                height,
                getLongSideAngleDegrees(corners),
                corners,
                isBoxClipped(left, right, bottom));
    }

    /**
     * Angle of the longest side of the rectangle, in degrees, from -90
     * (exclusive) to 90 (inclusive). A rectangle looks the same rotated 180
     * degrees, so the result is wrapped into that range.
     */
    private static double getLongSideAngleDegrees(List<TargetCorner> corners) {
        double longestLength = -1.0;
        double longestAngle = 0.0;

        for (int i = 0; i < 4; i++) {
            TargetCorner start = corners.get(i);
            TargetCorner end = corners.get((i + 1) % 4);

            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double length = Math.hypot(dx, dy);

            if (length > longestLength) {
                longestLength = length;
                longestAngle = Math.toDegrees(Math.atan2(dy, dx));
            }
        }

        while (longestAngle > 90.0) {
            longestAngle -= 180.0;
        }
        while (longestAngle <= -90.0) {
            longestAngle += 180.0;
        }
        return longestAngle;
    }

    // True if the box runs into the left, right or bottom edge of the image
    private static boolean isBoxClipped(double left, double right, double bottom) {
        double width = ObjectDetectionConstants.imageWidthPixels;
        double height = ObjectDetectionConstants.imageHeightPixels;
        double margin = ObjectDetectionConstants.edgeMarginPixels;

        if (width <= 0.0 || height <= 0.0) {
            return false;
        }
        return left <= margin
                || right >= width - margin
                || bottom >= height - margin;
    }

    //
    // Field position
    //

    /**
     * Casts a ray through the bottom-center of the box, intersects it with the
     * floor, and pushes the point back by the object radius.
     *
     * The camera pose on the field is the robot pose transformed by the
     * constant robotToCamera. Camera frame is WPILib style: x forward, y left,
     * z up.
     */
    private static Optional<LocatedObject> locateObject(
            Detection detection,
            Pose2d robotPose,
            double timestampSeconds) {
        if (detection.isClipped()) {
            return Optional.empty();
        }

        double imageWidth = ObjectDetectionConstants.imageWidthPixels;
        double imageHeight = ObjectDetectionConstants.imageHeightPixels;
        double horizontalFov = ObjectDetectionConstants.horizontalFovDegrees;
        double verticalFov = ObjectDetectionConstants.verticalFovDegrees;

        boolean sizeIsSet = imageWidth > 0.0 && imageHeight > 0.0;
        boolean fovIsSet = horizontalFov > 0.0 && verticalFov > 0.0
                && horizontalFov < 180.0 && verticalFov < 180.0;
        if (!sizeIsSet || !fovIsSet) {
            return Optional.empty();
        }

        // Focal lengths in pixels from the field of view
        double focalX = (imageWidth / 2.0) / Math.tan(Math.toRadians(horizontalFov / 2.0));
        double focalY = (imageHeight / 2.0) / Math.tan(Math.toRadians(verticalFov / 2.0));

        // Bottom-center pixel relative to the image center (right and up positive)
        double normalizedX = (detection.getCenterX() - imageWidth / 2.0) / focalX;
        double normalizedY = (imageHeight / 2.0 - detection.getBottomY()) / focalY;

        // Camera pose on the field, from the constant mounting transform
        Pose3d cameraPose = new Pose3d(robotPose)
                .transformBy(ObjectDetectionConstants.robotToCamera);

        // Ray direction in the field frame: forward 1, left -normalizedX, up normalizedY
        Translation3d ray = new Translation3d(1.0, -normalizedX, normalizedY)
                .rotateBy(cameraPose.getRotation());

        // The ray must point down to reach the floor
        if (ray.getZ() >= 0.0) {
            return Optional.empty();
        }

        double t = -cameraPose.getZ() / ray.getZ();
        if (!Double.isFinite(t) || t <= 0.0) {
            return Optional.empty();
        }

        Translation3d contact = cameraPose.getTranslation().plus(ray.times(t));

        // The contact point is on the near side of the object, so push it
        // directly away from the camera by the object radius to reach the center
        double dx = contact.getX() - cameraPose.getX();
        double dy = contact.getY() - cameraPose.getY();
        double length = Math.hypot(dx, dy);
        if (length < 1e-6) {
            return Optional.empty();
        }

        double radius = ObjectDetectionConstants.objectRadiusMeters;
        Translation2d center = new Translation2d(
                contact.getX() + radius * dx / length,
                contact.getY() + radius * dy / length);

        if (!Double.isFinite(center.getX()) || !Double.isFinite(center.getY())) {
            return Optional.empty();
        }

        Translation2d cameraXY = cameraPose.getTranslation().toTranslation2d();
        double distance = center.getDistance(cameraXY);

        return Optional.of(new LocatedObject(
                center,
                distance,
                detection.getClassId(),
                detection.getAspectRatio(),
                detection.getImageAngleDegrees(),
                timestampSeconds));
    }

    //
    // Camera and data health
    //

    public boolean isConnected() {
        return camera.isConnected();
    }

    public boolean isCorrectPipeline() {
        return !camera.getDriverMode()
                && camera.getPipelineIndex() == ObjectDetectionConstants.pipelineIndex;
    }

    /** Seconds since the newest frame, or infinity if there is none. */
    public double getFrameAgeSeconds() {
        double time = frameTime;
        if (!Double.isFinite(time)) {
            return Double.POSITIVE_INFINITY;
        }

        double age = Timer.getFPGATimestamp() - time;

        double tolerance = ObjectDetectionConstants.futureTimestampToleranceSeconds;
        if (!Double.isFinite(age) || age < -tolerance) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.max(0.0, age);
    }

    public boolean hasFreshFrame() {
        return cameraOk
                && getFrameAgeSeconds() <= ObjectDetectionConstants.staleTimeoutSeconds;
    }

    /** Capture time of the newest frame in FPGA seconds, or NaN if none. */
    public double getFrameTimeSeconds() {
        return frameTime;
    }

    //
    // Getting results
    //

    /** Every fresh image detection, lowest in the image first. Useful for debugging. */
    public List<Detection> getDetections() {
        if (!hasFreshFrame()) {
            return List.of();
        }
        return detections;
    }

    /** Every fresh located object, closest first. */
    public List<LocatedObject> getObjects() {
        if (!hasFreshFrame()) {
            return List.of();
        }
        return objects;
    }

    /** Fresh located objects of one class, closest first. */
    public List<LocatedObject> getObjects(int classId) {
        List<LocatedObject> matches = new ArrayList<>();
        for (LocatedObject object : getObjects()) {
            if (object.getClassId() == classId) {
                matches.add(object);
            }
        }
        return matches;
    }

    /** The closest fresh located object, or empty if none. */
    public Optional<LocatedObject> getBestObject() {
        List<LocatedObject> all = getObjects();
        return all.isEmpty() ? Optional.empty() : Optional.of(all.get(0));
    }

    /** The closest fresh located object of one class, or empty if none. */
    public Optional<LocatedObject> getBestObject(int classId) {
        List<LocatedObject> matches = getObjects(classId);
        return matches.isEmpty() ? Optional.empty() : Optional.of(matches.get(0));
    }

    public boolean hasObject() {
        return getBestObject().isPresent();
    }

    public boolean hasObject(int classId) {
        return getBestObject(classId).isPresent();
    }

    /** Distance from the camera to the closest object in meters, or empty. */
    public OptionalDouble getBestObjectDistanceMeters() {
        Optional<LocatedObject> best = getBestObject();
        return best.isEmpty()
                ? OptionalDouble.empty()
                : OptionalDouble.of(best.get().getDistanceFromCameraMeters());
    }

    //
    // Dashboard
    //

    private double getDistanceOrMinusOne() {
        return getBestObjectDistanceMeters().orElse(-1.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Object Detector");

        builder.addBooleanProperty("Has Object", this::hasObject, null);
        builder.addDoubleProperty("Object Count", () -> getObjects().size(), null);
        builder.addDoubleProperty("Object Distance Meters", this::getDistanceOrMinusOne, null);
    }
}