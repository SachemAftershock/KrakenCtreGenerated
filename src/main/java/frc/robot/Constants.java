package frc.robot;

public class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort1 = 0;
    public static final int kDriverControllerPort2 = 0;
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
}