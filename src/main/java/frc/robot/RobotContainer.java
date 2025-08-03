// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.ElevatorSubsystem;
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
    private ElevatorSubsystem mElevatorSubsystem = ElevatorSubsystem.getInstance();
    private AlgaePushSubsystem mAlgaePushSubsystem = AlgaePushSubsystem.getInstance();

    // kSpeedAt12Volts desired top speed
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController primaryController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort1);
    private final CommandXboxController secondaryController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(primaryController.getLeftY() * MaxSpeed)
                        .withVelocityY(primaryController.getLeftX() * MaxSpeed)
                        .withRotationalRate(-primaryController.getRightX() * MaxAngularRate)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        primaryController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        primaryController.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-primaryController.getLeftY(), -primaryController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        primaryController.back().and(primaryController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        primaryController.back().and(primaryController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        primaryController.start().and(primaryController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        primaryController.start().and(primaryController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        primaryController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        primaryController.leftTrigger().onTrue(MoveToPoseCommand.fromXYW(drivetrain, 1.0, 1.0, 45.0, false));
        primaryController.rightTrigger().onTrue(MoveToPoseCommand.fromXYW(drivetrain, 0.0, 0.0, 0.0, true));

        /*
         * Secondary Controller Commands
         */

        // Nudge Elevator Height up and down
        secondaryController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.85)
                .onTrue(new NudgeElevatorCommand(mElevatorSubsystem, 0.035));
        secondaryController.axisLessThan(XboxController.Axis.kLeftY.value, -0.85)
                .onTrue(new NudgeElevatorCommand(mElevatorSubsystem, -0.15));

        // Move Elevator Height
        secondaryController.povDown().whileTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eReceive));
        secondaryController.povLeft().whileTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL1));
        secondaryController.povUp().whileTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL2));
        secondaryController.povRight().whileTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL3));

        secondaryController.leftTrigger().whileTrue(new CoralGrappleCommand(mElevatorSubsystem, true));
        secondaryController.rightTrigger().whileTrue(new CoralGrappleCommand(mElevatorSubsystem, false));

        secondaryController.a().whileTrue(new PushOutCommand(mAlgaePushSubsystem));
        secondaryController.b().whileTrue(new PushInCommand(mAlgaePushSubsystem));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
