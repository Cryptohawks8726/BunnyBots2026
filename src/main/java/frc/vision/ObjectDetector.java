package frc.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Constants.ObjectDetectionConstants;

/**
 * Reads object detections from a PhotonVision camera, removes bad ones, and
 * keeps the rest sorted with the biggest (closest) detection first. It can also
 * estimate where a detected game piece is relative to the camera.
 *
 * <p>
 * Call update() once per robot loop, then use the getters.
 */
public final class ObjectDetector implements Sendable {

    private final PhotonCamera camera;

    // Valid detections (every class) from the newest frame, best first
    private List<Detection> targets = new ArrayList<>();

    // Capture time of the newest frame
    private double frameTime = Double.NaN;

    //
    // Data classes
    //

    /**
     * One detected object.
     *
     * <p>
     * Yaw is positive left and pitch is positive up (PhotonVision convention).
     * Center, width, height and area are in image pixels, with the origin at the
     * top-left and y pointing down.
     */
    public static final class Detection {
        private final int classId;
        private final double confidence;
        private final double yawDegrees;
        private final double pitchDegrees;
        private final double centerX;
        private final double centerY;
        private final double width;
        private final double height;
        private final double orientationDegrees;

        public Detection(
                int classId,
                double confidence,
                double yawDegrees,
                double pitchDegrees,
                double centerX,
                double centerY,
                double width,
                double height,
                double orientationDegrees) {
            this.classId = classId;
            this.confidence = confidence;
            this.yawDegrees = yawDegrees;
            this.pitchDegrees = pitchDegrees;
            this.centerX = centerX;
            this.centerY = centerY;
            this.width = width;
            this.height = height;
            this.orientationDegrees = orientationDegrees;
        }

        public int getClassId() {
            return classId;
        }

        public double getConfidence() {
            return confidence;
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

        /**
         * Angle of the box's long side. YOLO detection boxes are axis-aligned
         * (PhotonVision's min-area rectangle of one is the box itself), so this
         * is really just 0 (wider than tall) or 90 (taller than wide).
         */
        public double getOrientationDegrees() {
            return orientationDegrees;
        }

        public double getArea() {
            return width * height;
        }
    }

    /** A game piece located relative to the camera. */
    public static final class GamePiece {
        private final Translation2d positionRelativeToCamera;
        private final double heightMeters;
        private final double orientationDegrees;

        public GamePiece(
                Translation2d positionRelativeToCamera,
                double heightMeters,
                double orientationDegrees) {
            this.positionRelativeToCamera = positionRelativeToCamera;
            this.heightMeters = heightMeters;
            this.orientationDegrees = orientationDegrees;
        }

        /** Forward (x) and left (y) distance from the camera, in meters. */
        public Translation2d getPositionRelativeToCamera() {
            return positionRelativeToCamera;
        }

        /** Height of the piece's center above the floor, in meters. */
        public double getHeightMeters() {
            return heightMeters;
        }

        public double getOrientationDegrees() {
            return orientationDegrees;
        }

        /** Straight-line distance from the camera, in meters. */
        public double getDistanceMeters() {
            return positionRelativeToCamera.getNorm();
        }
    }

    // Sorts the biggest box first, then the most confident
    private static final class BiggestFirst implements Comparator<Detection> {
        @Override
        public int compare(Detection a, Detection b) {
            int byArea = Double.compare(b.getArea(), a.getArea());
            if (byArea != 0) {
                return byArea;
            }
            return Double.compare(b.getConfidence(), a.getConfidence());
        }
    }

    private static final Comparator<Detection> BIGGEST_FIRST = new BiggestFirst();

    //
    // Setup and updating
    //

    public ObjectDetector() {
        camera = new PhotonCamera(ObjectDetectionConstants.cameraName);
        camera.setDriverMode(false);
        camera.setPipelineIndex(ObjectDetectionConstants.pipelineIndex);
    }

    /** Reads new camera data. Call once per robot loop. */
    public void update() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        // Throw everything away if the camera is unusable
        if (!isConnected() || !isCorrectPipeline()) {
            clear();
            return;
        }

        // Only the newest result matters
        if (!results.isEmpty()) {
            loadResult(results.get(results.size() - 1));
        }

        if (!hasFreshFrame()) {
            targets = new ArrayList<>();
        }
    }

    private void loadResult(PhotonPipelineResult result) {
        frameTime = result.getTimestampSeconds();

        List<Detection> found = new ArrayList<>();

        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                Detection detection = convertTarget(target);
                if (detection != null) {
                    found.add(detection);
                }
            }
        }

        found.sort(BIGGEST_FIRST);
        targets = found;
    }

    private void clear() {
        targets = new ArrayList<>();
        frameTime = Double.NaN;
    }

    //
    // Turning a PhotonVision target into a Detection
    //

    /** Returns the Detection, or null if the target should be ignored. */
    private Detection convertTarget(PhotonTrackedTarget target) {
        int classId = target.getDetectedObjectClassID();
        double confidence = target.getDetectedObjectConfidence();

        if (classId < 0 || !isConfidenceUsable(confidence)) {
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

        // Find the box edges from the corners
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
                confidence,
                yaw,
                pitch,
                left + width / 2.0,
                top + height / 2.0,
                width,
                height,
                getLongSideAngleDegrees(corners));
    }

    private boolean isConfidenceUsable(double confidence) {
        if (!Double.isFinite(confidence)) {
            return false;
        }
        if (confidence < 0.0 || confidence > 1.0) {
            return false;
        }
        return confidence >= ObjectDetectionConstants.minConfidence;
    }

    /**
     * Angle of the box's longest side in image space, in degrees from -90
     * (exclusive) to 90 (inclusive), measured from the image x-axis.
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

        // A box looks the same rotated 180 degrees, so wrap into (-90, 90]
        while (longestAngle > 90.0) {
            longestAngle -= 180.0;
        }
        while (longestAngle <= -90.0) {
            longestAngle += 180.0;
        }

        return longestAngle;
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
        if (!Double.isFinite(frameTime)) {
            return Double.POSITIVE_INFINITY;
        }

        double age = Timer.getFPGATimestamp() - frameTime;

        // A timestamp from the future means something is wrong
        double tolerance = ObjectDetectionConstants.futureTimestampToleranceSeconds;
        if (!Double.isFinite(age) || age < -tolerance) {
            return Double.POSITIVE_INFINITY;
        }

        return Math.max(0.0, age);
    }

    public boolean hasFreshFrame() {
        return isConnected()
                && isCorrectPipeline()
                && getFrameAgeSeconds() <= ObjectDetectionConstants.staleTimeoutSeconds;
    }

    //
    // Getting detections
    //

    /**
     * Fresh detections, best first. If ObjectDetectionConstants.targetClassId
     * is 0 or higher, only that class is returned. Empty if the data is stale.
     */
    public List<Detection> getDetections() {
        int wantedClassId = ObjectDetectionConstants.targetClassId;

        // A negative setting means "all classes"
        if (wantedClassId < 0) {
            return getAllFreshDetections();
        }
        return getDetections(wantedClassId);
    }

    /**
     * Fresh detections of one class, best first. This ignores
     * ObjectDetectionConstants.targetClassId, so you can always ask for a
     * specific class.
     */
    public List<Detection> getDetections(int classId) {
        List<Detection> matches = new ArrayList<>();

        for (Detection detection : getAllFreshDetections()) {
            if (detection.getClassId() == classId) {
                matches.add(detection);
            }
        }

        return matches;
    }

    // Every class, but only if the data is fresh
    private List<Detection> getAllFreshDetections() {
        if (!hasFreshFrame()) {
            return new ArrayList<>();
        }
        return new ArrayList<>(targets);
    }

    /** The largest fresh detection, or empty if there is none. */
    public Optional<Detection> getBestTarget() {
        List<Detection> detections = getDetections();

        if (detections.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(detections.get(0));
    }

    /** The largest fresh detection of one class, or empty if there is none. */
    public Optional<Detection> getBestTarget(int classId) {
        List<Detection> detections = getDetections(classId);

        if (detections.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(detections.get(0));
    }

    public boolean hasTarget() {
        return getBestTarget().isPresent();
    }

    public boolean hasTarget(int classId) {
        return getBestTarget(classId).isPresent();
    }

    public int getTargetCount() {
        return getDetections().size();
    }

    //
    // Distance and position
    //

    /**
     * Estimates the floor distance from the camera to a detection, in meters.
     *
     * <p>
     * Assumes the object's center is at targetHeightMeters. Camera pitch and
     * target pitch are both positive up, so their sum is negative when looking
     * down. Distance = (cameraHeight - targetHeight) / tan(angleDown).
     *
     * @return the distance, or empty if the camera is not looking down at the
     *         object
     */
    public static Optional<Double> estimateDistanceMeters(Detection detection) {
        double cameraHeight = ObjectDetectionConstants.cameraHeightMeters;
        double targetHeight = ObjectDetectionConstants.targetHeightMeters;

        double totalPitch = ObjectDetectionConstants.cameraPitchDegrees + detection.getPitchDegrees();
        double angleDownDegrees = -totalPitch;

        if (cameraHeight <= targetHeight || angleDownDegrees <= 0.0) {
            return Optional.empty();
        }

        double distance = (cameraHeight - targetHeight) / Math.tan(Math.toRadians(angleDownDegrees));

        if (!Double.isFinite(distance) || distance < 0.0) {
            return Optional.empty();
        }

        return Optional.of(distance);
    }

    /** Distance to the best target in meters, or empty if there is none. */
    public Optional<Double> getBestTargetDistanceMeters() {
        Optional<Detection> best = getBestTarget();

        if (best.isEmpty()) {
            return Optional.empty();
        }
        return estimateDistanceMeters(best.get());
    }

    /**
     * Locates the best game piece relative to the camera, using the camera
     * mounting constants. cameraPitchDegrees is positive up, but the math below
     * wants a tilt that is positive down, so the sign is flipped here.
     */
    public Optional<GamePiece> getBestGamePiece() {
        return getBestGamePiece(
                ObjectDetectionConstants.cameraHeightMeters,
                -ObjectDetectionConstants.cameraPitchDegrees);
    }

    /**
     * Locates the best game piece relative to the camera.
     *
     * @param cameraHeightMeters     height of the camera lens above the floor
     * @param cameraAngleDownDegrees how far the camera is tilted down from
     *                               horizontal
     * @return the piece's position, or empty if there is no target or the
     *         camera calibration constants are not set
     */
    public Optional<GamePiece> getBestGamePiece(
            double cameraHeightMeters, double cameraAngleDownDegrees) {
        Optional<Detection> best = getBestTarget();

        if (best.isEmpty()) {
            return Optional.empty();
        }
        return locateGamePiece(best.get(), cameraHeightMeters, cameraAngleDownDegrees);
    }

    /**
     * Turns a box center in the image into a floor position by drawing a ray
     * from the camera through that pixel and finding where it hits the height
     * of the piece's center.
     *
     * <p>
     * Coordinates: x is forward from the camera, y is left, z is up.
     */
    private static Optional<GamePiece> locateGamePiece(
            Detection detection,
            double cameraHeightMeters,
            double cameraAngleDownDegrees) {
        double imageWidth = ObjectDetectionConstants.imageWidthPixels;
        double imageHeight = ObjectDetectionConstants.imageHeightPixels;
        double horizontalFov = ObjectDetectionConstants.horizontalFovDegrees;
        double verticalFov = ObjectDetectionConstants.verticalFovDegrees;

        // Calibration constants must be filled in
        boolean sizeIsSet = imageWidth > 0.0 && imageHeight > 0.0;
        boolean fovIsSet = horizontalFov > 0.0 && verticalFov > 0.0
                && horizontalFov < 180.0 && verticalFov < 180.0;
        if (!sizeIsSet || !fovIsSet) {
            return Optional.empty();
        }

        // Focal lengths in pixels, from the field of view
        double focalX = (imageWidth / 2.0) / Math.tan(Math.toRadians(horizontalFov / 2.0));
        double focalY = (imageHeight / 2.0) / Math.tan(Math.toRadians(verticalFov / 2.0));

        // Box center relative to the image center (right and up are positive)
        double normalizedX = (detection.getCenterX() - imageWidth / 2.0) / focalX;
        double normalizedY = (imageHeight / 2.0 - detection.getCenterY()) / focalY;

        // Ray through that pixel, with the camera tilted down
        double tilt = Math.toRadians(cameraAngleDownDegrees);
        double rayForward = Math.cos(tilt) + normalizedY * Math.sin(tilt);
        double rayLeft = -normalizedX;
        double rayUp = -Math.sin(tilt) + normalizedY * Math.cos(tilt);

        // The ray has to point downward to hit the floor
        if (rayUp >= 0.0) {
            return Optional.empty();
        }

        // How far along the ray to reach the piece's height
        double pieceHeight = ObjectDetectionConstants.targetHeightMeters;
        double scale = (pieceHeight - cameraHeightMeters) / rayUp;

        if (scale <= 0.0) {
            return Optional.empty();
        }

        double forward = scale * rayForward;
        double left = scale * rayLeft;

        if (!Double.isFinite(forward) || !Double.isFinite(left)) {
            return Optional.empty();
        }

        Translation2d position = new Translation2d(forward, left);
        return Optional.of(new GamePiece(position, pieceHeight, detection.getOrientationDegrees()));
    }

    //
    // Dashboard
    //

    private double getDistanceOrMinusOne() {
        Optional<Double> distance = getBestTargetDistanceMeters();

        if (distance.isEmpty()) {
            return -1.0;
        }
        return distance.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Object Detector");

        builder.addBooleanProperty("Has Target", () -> hasTarget(), null);

        // Shows -1 when there is no target
        builder.addDoubleProperty(
                "Target Distance Meters",
                () -> getDistanceOrMinusOne(),
                null);
    }
}