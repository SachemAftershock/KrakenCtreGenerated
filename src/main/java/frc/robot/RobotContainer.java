// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaePushSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.enums.ElevatorPosEnum;
import frc.robot.commands.Elevator.NudgeElevatorCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.commands.Elevator.CoralGrappleCommand;
import frc.robot.commands.Elevator.MoveToHeightCommand;
import frc.robot.commands.AlgaePush.PushInCommand;
import frc.robot.commands.AlgaePush.PushOutCommand;
import frc.robot.commands.Auto.MoveToPoseCommand;

public class RobotContainer {
    public final ElevatorSubsystem mElevatorSubsystem = ElevatorSubsystem.getInstance();
    public final AlgaePushSubsystem mAlgaePushSubsystem = AlgaePushSubsystem.getInstance();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSubsystem mVisionSubsystem = VisionSubsystem.getInstance();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController primaryController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort1);
    private final CommandXboxController secondaryController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort2);

    private final PathPlannerAuto AutoTest;

    public RobotContainer() {

        Command goToL2 = new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL2);
        NamedCommands.registerCommand("L2", goToL2);
        AutoTest = new PathPlannerAuto("test_auto_1");

        Pose2d temp = null;
        double timestamp = 10;
        Matrix<N3, N1> visionMeasurementStdDevs;
        drivetrain.addVisionMeasurement(temp, timestamp, visionMeasurementStdDevs);
        mVisionSubsystem.bindEstimateAcceptor(drivetrain::addVisionMeasurement);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(primaryController.getLeftY() * MaxSpeed)
                        .withVelocityY(primaryController.getLeftX() * MaxSpeed)
                        .withRotationalRate(-primaryController.getRightX() * MaxAngularRate)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        primaryController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        primaryController.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-primaryController.getLeftY(),
                        -primaryController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // primaryController.back().and(primaryController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // primaryController.back().and(primaryController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // primaryController.start().and(primaryController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // primaryController.start().and(primaryController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        primaryController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        primaryController.leftTrigger().whileTrue(
                new MoveToPoseCommand(drivetrain, new Pose2d(1.0, 1.0, new Rotation2d(45)), false));
        primaryController.rightTrigger().whileTrue(
                new MoveToPoseCommand(drivetrain, new Pose2d(0.0, 0.0, new Rotation2d(0)), true));

        /*
         * Secondary Controller Commands
         */

        // Nudge Elevator Height up and down
        secondaryController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.85)
                .onTrue(new NudgeElevatorCommand(mElevatorSubsystem, 0.035));
        secondaryController.axisLessThan(XboxController.Axis.kLeftY.value, -0.85)
                .onTrue(new NudgeElevatorCommand(mElevatorSubsystem, -0.15));

        // Move Elevator Height
        secondaryController.povDown()
                .whileTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eReceive));
        secondaryController.povLeft()
                .whileTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL1));
        secondaryController.povUp().whileTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL2));
        secondaryController.povRight()
                .whileTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL3));

        secondaryController.leftTrigger().whileTrue(new CoralGrappleCommand(mElevatorSubsystem, true));
        secondaryController.rightTrigger().whileTrue(new CoralGrappleCommand(mElevatorSubsystem, false));

        secondaryController.a().whileTrue(new PushOutCommand(mAlgaePushSubsystem));
        secondaryController.b().whileTrue(new PushInCommand(mAlgaePushSubsystem));

    }

    public Command getAutonomousCommand() {
        System.out.println("why");
        return AutoTest;
    }
}
