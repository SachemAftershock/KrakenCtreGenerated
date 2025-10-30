package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort1 = 0;
        public static final int kDriverControllerPort2 = 1;
    }

    public static class ElevatorConstants {
        public static final int kElevatorMotorID = 21;
        public static final int kGrappleMotorID = 24;
        public static final int kTopLimitSwitchID = 15;
        public static final int kBottomLimitSwitchID = 16;
        public static final int kBeamBreakerID = 17;
        public static final int kElevatorLaserCanID = 30;
        public static final int kGrappleLaserCanID = 31;
        public static final int kROIx = 0;
        public static final int kROIy = 0;
        public static final int kROIw = 0;
        public static final int kROIh = 0;
        public static final double kCoralInLaserCanDistance = 50;
        public static final int kStickTranslation = 0;
        public static final double kTolerance = 0.02;
        public static final double kMaxSpeed = 0.5;
        public static final int kGroundHeight = 0;
        public static final int kL1Height = 0;
        public static final int kL2Height = 0;
        public static final int kL3Height = 0;
    }

    public static class PusherConstants {
        public static final int kPushmotorID = 22;
        public static final double kTolerance = 0.5;
    }

    public static enum CardinalDirection {
        eX, eY
    }

    public static class VisionConstants {
        public static final String kFLCamName = "FL_CAM";
        public static final Transform3d kFLCamTransform =
                // CHECK THE SIGNS
                // This is formatted as X (forward + / backwards - ), Y (left + / right -), Z (up + / down -)
                // Roll, Pitch, Yaw (CCW is positive)
                new Transform3d(
                        Meters.of(0.25), Meters.of(0.25), Meters.of(0.0),
                        new Rotation3d(Degrees.of(0), Degrees.of(45), Degrees.of(-45))
                );
    
        public static final String kFRCamName = "FR_CAM";
        public static final Transform3d kFRCamTransform =
                new Transform3d(
                        Meters.of(0.25), Meters.of(-0.25), Meters.of(0.0),
                        new Rotation3d(Degrees.of(0), Degrees.of(45), Degrees.of(45))
                );
    
        public static final String kBLCamName = "BL_CAM";
        public static final Transform3d kBLCamTransform =
                new Transform3d(
                        Meters.of(-0.25), Meters.of(0.25), Meters.of(0.0),
                        new Rotation3d(Degrees.of(0), Degrees.of(45), Degrees.of(-135))
                );
    
        public static final String kBRCamName = "BR_CAM";
        public static final Transform3d kBRCamTransform =
                new Transform3d(
                        Meters.of(-0.25), Meters.of(-0.25), Meters.of(0.0),
                        new Rotation3d(Degrees.of(0), Degrees.of(45), Degrees.of(135))
                );

        public static String[] kCameraNames = {
            kFLCamName, kFRCamName, kBLCamName, kBRCamName
        };

        public static Transform3d[] kCameraTransforms = {
            kFLCamTransform, kFRCamTransform, kBLCamTransform, kBRCamTransform
        };

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}