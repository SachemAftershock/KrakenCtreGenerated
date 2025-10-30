package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import static frc.robot.Constants.VisionConstants.kCameraNames;
import static frc.robot.Constants.VisionConstants.kCameraTransforms;
import static frc.robot.Constants.VisionConstants.kSingleTagStdDevs;
import static frc.robot.Constants.VisionConstants.kMultiTagStdDevs;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class VisionSubsystem implements Subsystem {
    private static VisionSubsystem mInstance;

    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] photonEstimators;
    private List<Matrix<N3, N1>> curStdDevs;
    private EstimateConsumer estimateConsumer;

    public VisionSubsystem() {
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        cameras = new PhotonCamera[kCameraNames.length];
        photonEstimators = new PhotonPoseEstimator[kCameraNames.length];
        curStdDevs = IntStream.range(0, kCameraNames.length)
                .mapToObj(i -> kSingleTagStdDevs)
                .collect(Collectors.toList());

        for (int i = 0; i < kCameraNames.length; i++) {
            cameras[i] = new PhotonCamera(kCameraNames[i]);
            photonEstimators[i] = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    kCameraTransforms[i]);
            photonEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (int i = 0; i < kCameraNames.length; i++) {
            for (var change : cameras[i].getAllUnreadResults()) {
                visionEst = photonEstimators[i].update(change);
                updateEstimationStdDevs(visionEst, change.getTargets(), i);

                if (visionEst.isPresent()) {
                    var est = visionEst.get();
                    var estStdDevs = getEstimationStdDevs(i);
                    estimateConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                };
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs(int i) {
        return curStdDevs.get(i);
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, int i) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs.set(i, kSingleTagStdDevs);

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimators[i].getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs.set(i, kSingleTagStdDevs);
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs.set(i, estStdDevs);
            }
        }
    }

    public synchronized static VisionSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new VisionSubsystem();
        }
        return mInstance;
    }

    public void bindEstimateAcceptor(EstimateConsumer addVisionMeasurement) {
        estimateConsumer = addVisionMeasurement;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
