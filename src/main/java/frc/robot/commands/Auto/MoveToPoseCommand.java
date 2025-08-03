package frc.robot.commands.Auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.RobotContainer;

public class MoveToPoseCommand extends Command {
    private final CommandSwerveDrivetrain mDrivetrain;
    private final Transform2d mDeltaPose;
    private Pose2d mStartPose;
    private Pose2d mTargetPose;
    private Pose2d currentPose;
    
    private double positionTolerance = 0.05;
    private double rotationTolerance = 0.05;

    private final boolean isAbsolute;

    private PIDController xPID;
    private PIDController yPID;
    private PIDController wPID;

    private double xSpeed = 0.0;
    private double ySpeed = 0.0;
    private double wSpeed = 0.0;

    private double currentX = 0.0;
    private double currentY = 0.0;
    private double currentW = 0.0;

    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetW = 0.0;

    private final double maxSpeed = 2;
    private final double maxAngularRate = 0.5;

    private final double[] translatePIDs = { 5, 0.0, 1 };
    private final double[] rotatePIDs = { 1, 0.0, 0.2 };

    boolean positionReached = false;
    boolean rotationReached = false;

    private final SwerveRequest.FieldCentric moveCommand = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.Idle idleCommand = new SwerveRequest.Idle();

    public MoveToPoseCommand(CommandSwerveDrivetrain drivetrain, Pose2d deltaPose, boolean isAbsolute) {
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);

        // Initialize for either absolute or relative
        mTargetPose = deltaPose;
        
        if (isAbsolute) {
            mTargetPose = deltaPose;
            mDeltaPose = null;
        } else {
            mDeltaPose = new Transform2d(deltaPose.getTranslation(), deltaPose.getRotation());
            mTargetPose = null;
        }

        this.isAbsolute = isAbsolute;

        xPID = new PIDController(translatePIDs[0], translatePIDs[1], translatePIDs[2]);
        yPID = new PIDController(translatePIDs[0], translatePIDs[1], translatePIDs[2]);
        wPID = new PIDController(rotatePIDs[0], rotatePIDs[1], rotatePIDs[2]);
    }

    public static MoveToPoseCommand fromX(CommandSwerveDrivetrain drivetrain, double x, boolean isAbsolute) {
        return new MoveToPoseCommand(drivetrain, new Pose2d(x, 0, new Rotation2d(0)), isAbsolute);
    }

    public static MoveToPoseCommand fromY(CommandSwerveDrivetrain drivetrain, double y, boolean isAbsolute) {
        return new MoveToPoseCommand(drivetrain, new Pose2d(0, y, new Rotation2d(0)), isAbsolute);
    }

    public static MoveToPoseCommand fromW(CommandSwerveDrivetrain drivetrain, double w, boolean isAbsolute) {
        return new MoveToPoseCommand(drivetrain, new Pose2d(0, 0, new Rotation2d(w)), isAbsolute);
    }

    public static MoveToPoseCommand fromXY(CommandSwerveDrivetrain drivetrain, double x, double y, boolean isAbsolute) {
        return new MoveToPoseCommand(drivetrain, new Pose2d(x, y, new Rotation2d(0)), isAbsolute);
    }

    public static MoveToPoseCommand fromXW(CommandSwerveDrivetrain drivetrain, double x, double w, boolean isAbsolute) {
        return new MoveToPoseCommand(drivetrain, new Pose2d(x, 0, new Rotation2d(w)), isAbsolute);
    }

    public static MoveToPoseCommand fromYW(CommandSwerveDrivetrain drivetrain, double y, double w, boolean isAbsolute) {
        return new MoveToPoseCommand(drivetrain, new Pose2d(0, y, new Rotation2d(w)), isAbsolute);
    }

    public static MoveToPoseCommand fromXYW(CommandSwerveDrivetrain drivetrain, double x, double y,
            double w, boolean isAbsolute) {
        return new MoveToPoseCommand(drivetrain, new Pose2d(x, y, new Rotation2d(w)), isAbsolute);
    }

    @Override
    public void initialize() {
        mStartPose = mDrivetrain.getState().Pose;
        if (!isAbsolute) {
            mTargetPose = mStartPose.plus(mDeltaPose);
        }
    }

    @Override
    public void execute() {
        currentPose = mDrivetrain.getState().Pose;

        currentX = mDrivetrain.getState().Pose.getX();
        currentY = mDrivetrain.getState().Pose.getY();
        currentW = mDrivetrain.getState().Pose.getRotation().getRadians();

        targetX = mTargetPose.getX();
        targetY = mTargetPose.getY();
        targetW = mTargetPose.getRotation().getRadians();

        xSpeed = xPID.calculate(currentX, targetX);
        ySpeed = yPID.calculate(currentY, targetY);
        wSpeed = wPID.calculate(currentW, targetW);

        SmartDashboard.putNumber("xSpeed Unclamped", xSpeed);
        SmartDashboard.putNumber("ySpeed Unclamped", ySpeed);
        SmartDashboard.putNumber("wSpeed Unclamped", wSpeed);

        xSpeed = MathUtil.clamp(xSpeed, -maxSpeed, maxSpeed);
        ySpeed = MathUtil.clamp(ySpeed, -maxSpeed, maxSpeed);
        wSpeed = MathUtil.clamp(wSpeed, -maxAngularRate, maxAngularRate);

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("wSpeed", wSpeed);

        // mDrivetrain.setControl(moveCommand.withVelocityX(1));
        mDrivetrain.setControl(moveCommand.withVelocityX(-xSpeed).withVelocityY(-ySpeed).withRotationalRate(wSpeed));
    }

    @Override
    public boolean isFinished() {
        positionReached = currentPose.getTranslation().getDistance(mTargetPose.getTranslation()) < positionTolerance;
        rotationReached = Math.abs(currentPose.getRotation().getRadians() - mTargetPose.getRotation().getRadians()) < rotationTolerance;
        
        SmartDashboard.putBoolean("Position Reached", positionReached);
        SmartDashboard.putBoolean("Rotation Reached", rotationReached);
        boolean out = positionReached && rotationReached;
        SmartDashboard.putBoolean("Done", out);
        return out;
        // return positionReached && rotationReached;
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.applyRequest(() -> idleCommand);
    }
}
