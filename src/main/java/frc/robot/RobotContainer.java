// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.AftershockCommandXboxController;
import frc.robot.commands.AlgaePush.PushInCommand;
import frc.robot.commands.AlgaePush.PushOutCommand;
import frc.robot.commands.Auto.DelayCommand;
import frc.robot.commands.Auto.MoveToPoseCommand;
import frc.robot.commands.Elevator.CoralGrappleCommand;
import frc.robot.commands.Elevator.MoveToHeightCommand;
import frc.robot.commands.Elevator.NudgeElevatorCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.enums.ElevatorPosEnum;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaePushSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    private ElevatorSubsystem mElevatorSubsystem = ElevatorSubsystem.getInstance();
    private AlgaePushSubsystem mAlgaePushSubsystem = AlgaePushSubsystem.getInstance();
    
    private Command elevator_L1 = new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL1);
    private Command elevator_L2 = new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL2);
    private Command elevator_Receive = new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eReceive);
    private Command elevator_Stowed = new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eStowed);
    private Command coral_Eject = new CoralGrappleCommand(mElevatorSubsystem, false);
    private Command coral_Receive = new CoralGrappleCommand(mElevatorSubsystem, true);
    
//     private Command sequenceBlue4CoralCommand = new SequentialCommandGroup(
//         (new ParallelCommandGroup(
//                 (new PathPlannerAuto("001.1_Blue_Outside_to_Reef_J")),
//                 (new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL2))).andThen
//         (new DelayCommand(0.1)).andThen
//         (new CoralGrappleCommand(mElevatorSubsystem, false) ).andThen // eject Coral
//         (new DelayCommand(0.45)).andThen
//         (new ParallelCommandGroup(
//                 (new PathPlannerAuto("001.2_Blue_Reef_J_to_Human_Player")),
//                 (new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eReceive))
//                 )).andThen
//         (new CoralGrappleCommand(mElevatorSubsystem, true)).andThen // intake Coral
//         (new ParallelCommandGroup(
//                 (new PathPlannerAuto("001.3_Human_Player_to_Blue_Reef_J")),
//                 (new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL1))
//                 )).andThen
//         (new CoralGrappleCommand(mElevatorSubsystem, false))).andThen // eject Coral
//         (new DelayCommand(0.45)).andThen
//         (new ParallelCommandGroup(
//                 (new PathPlannerAuto("001.2_Blue_Reef_J_to_Human_Player")),
//                 (new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eReceive))
//                 )).andThen
//         (new CoralGrappleCommand(mElevatorSubsystem, true)).andThen // intake Coral
//         (new ParallelCommandGroup(
//                 (new PathPlannerAuto("001.4_Human_Player_to_Blue_Reef_I")),
//                 (new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL2))
//                 )).andThen
//         (new CoralGrappleCommand(mElevatorSubsystem, false)).andThen // eject Coral
//         (new DelayCommand(0.45)).andThen
//         (new ParallelCommandGroup(
//                 (new PathPlannerAuto("001.5_Blue_Reef_I_to_Human_Player")),
//                 (new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eReceive))
//                 )).andThen
//         (new CoralGrappleCommand(mElevatorSubsystem, true)).andThen // intake Coral
//         (new ParallelCommandGroup(
//                 (new PathPlannerAuto("001.4_Human_Player_to_Blue_Reef_I")),
//                 (new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL1))
//                 )).andThen
//         (new CoralGrappleCommand(mElevatorSubsystem, false)) // eject Coral
//         ); 

    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    private PathConstraints constraints_PathFinder = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Load the path we want to pathfind to and follow
    private PathPlannerPath path_StageJI_to_J;
    private PathPlannerPath path_StageJI_to_I;

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    private Command pathfindingCommand_Anywhere_to_J;
    private Command pathfindingCommand_Anywhere_to_I;

    // kSpeedAt12Volts desired top speed
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a deadband, keep it tight for precision control (no jerk)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final AftershockCommandXboxController primaryController = new AftershockCommandXboxController(
            OperatorConstants.kDriverControllerPort1);
    private final AftershockCommandXboxController secondaryController = new AftershockCommandXboxController(
            OperatorConstants.kDriverControllerPort2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //private final PathPlannerAuto AutoTest;
    
    private final Command Auto_001_Blue_Outside_Coral_J2J1I2I1;
    private final Command Auto_002_Blue_Move_Somewhere;

    private final boolean enableSysIdControls = false;

    public RobotContainer() {
        
        // Command goToL2 = new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL2);
        // NamedCommands.registerCommand("L2", goToL2);
        // AutoTest = new PathPlannerAuto("test_auto_1");

        NamedCommands.registerCommand("elevator_L1", elevator_L1);
        NamedCommands.registerCommand("elevator_L2", elevator_L2);
        NamedCommands.registerCommand("elevator_Receive", elevator_Receive);
        NamedCommands.registerCommand("elevator_Stowed", elevator_Stowed);
        NamedCommands.registerCommand("coral_Eject", coral_Eject);
        NamedCommands.registerCommand("coral_Receive", coral_Receive);
        Auto_001_Blue_Outside_Coral_J2J1I2I1 = new PathPlannerAuto("Auto_001_Blue_Outside_Coral_J2J1I2I1");
        Auto_002_Blue_Move_Somewhere = new PathPlannerAuto("Auto_002_Blue_Move_Somewhere");
    
        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        String pathFile_To_J = "JI_Stage_to_J";
        try {
                path_StageJI_to_J = PathPlannerPath.fromPathFile(pathFile_To_J);
                pathfindingCommand_Anywhere_to_J = AutoBuilder.pathfindThenFollowPath(path_StageJI_to_J, constraints_PathFinder);
        } catch (Exception e) {
                System.out.println("Path File not found or failed to build path: '" + pathFile_To_J + "'");
                path_StageJI_to_J = null;
                pathfindingCommand_Anywhere_to_J = null;
                e.printStackTrace();
        }

        String pathFile_To_I = "JI_Stage_to_I";
        try {
                path_StageJI_to_I = PathPlannerPath.fromPathFile(pathFile_To_I);
                pathfindingCommand_Anywhere_to_I = AutoBuilder.pathfindThenFollowPath(path_StageJI_to_I, constraints_PathFinder);
        } catch (Exception e) {
                System.out.println("Path File not found or failed to build path: '" + pathFile_To_I + "'");
                path_StageJI_to_I = null;
                pathfindingCommand_Anywhere_to_I = null;
                e.printStackTrace();
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> 
                drive.withVelocityX(primaryController.getLeftY() * MaxSpeed)
                        .withVelocityY(primaryController.getLeftX() * MaxSpeed)
                        .withRotationalRate(-primaryController.getRightX() * MaxAngularRate)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        
        primaryController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        primaryController.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-primaryController.getLeftY(), -primaryController.getLeftX()))));

        
        if (enableSysIdControls) {
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
                //primaryController.back().and(primaryController.y()).and(enableSysIdControls).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                primaryController.back().and(primaryController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                primaryController.back().and(primaryController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                primaryController.start().and(primaryController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                primaryController.start().and(primaryController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        }

        // reset the field-centric heading on left bumper press
        primaryController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        primaryController.leftTrigger().onTrue(new MoveToPoseCommand(drivetrain, new Pose2d(1.0, 1.0, new Rotation2d(45)), false));
        primaryController.rightTrigger().onTrue(new MoveToPoseCommand(drivetrain, new Pose2d(0.0, 0.0, new Rotation2d(0)), false));

        if (pathfindingCommand_Anywhere_to_J != null) {
                System.out.print("pathfindingCommand_Anywhere_to_J not null");
                primaryController.povLeft().onTrue(pathfindingCommand_Anywhere_to_J);
        }

        if (pathfindingCommand_Anywhere_to_I != null) {
                System.out.print("pathfindingCommand_Anywhere_to_I not null");
                primaryController.povRight().onTrue(pathfindingCommand_Anywhere_to_I);
        }

        // Aim down field towards opposing alliance.  Stay at same field coordinate. 
        primaryController.povUp().onTrue(new MoveToPoseCommand(drivetrain, new Pose2d(drivetrain.getState().Pose.getTranslation().getX(), drivetrain.getState().Pose.getTranslation().getY(), new Rotation2d(0)), true));
        
        // Aim towards own alliance direction.  Stay at same field coordinate. 
        primaryController.povDown().onTrue(new MoveToPoseCommand(drivetrain, new Pose2d(drivetrain.getState().Pose.getTranslation().getX(), drivetrain.getState().Pose.getTranslation().getY(), new Rotation2d(180)), true));


// This was unneccessary, see secondary controller does it already.  May not need aftershockXboxController.  Remove it if not. 
        // Trigger ElevatorUpDPad = new Trigger (() -> primaryController.getDPadDown());
        // ElevatorUpDPad.whileTrue(new MoveToPoseCommand(drivetrain, new Pose2d(drivetrain.getState().Pose.getTranslation().getX(), drivetrain.getState().Pose.getTranslation().getY(), new Rotation2d(0)), true));

        // Trigger ElevatorDownDPad = new Trigger (() -> primaryController.getDPadDown());
        // ElevatorDownDPad.whileTrue(new MoveToPoseCommand(drivetrain, new Pose2d(drivetrain.getState().Pose.getTranslation().getX(), drivetrain.getState().Pose.getTranslation().getY(), new Rotation2d(180)), true));

        // Trigger ElevatorLeftDPad = new Trigger (() -> primaryController.getDPadDown());
        // ElevatorLeftDPad.whileTrue(new MoveToPoseCommand(drivetrain, new Pose2d(drivetrain.getState().Pose.getTranslation().getX(), drivetrain.getState().Pose.getTranslation().getY(), new Rotation2d(90)), true));

        // Trigger ElevatorRightDPad = new Trigger (() -> primaryController.getDPadDown());
        // ElevatorRightDPad.whileTrue(new MoveToPoseCommand(drivetrain, new Pose2d(drivetrain.getState().Pose.getTranslation().getX(), drivetrain.getState().Pose.getTranslation().getY(), new Rotation2d(-90)), true));

        /*
         * Secondary Controller Commands
         */

        // Nudge Elevator Height up and down
        secondaryController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.85)
                .onTrue(new NudgeElevatorCommand(mElevatorSubsystem, 0.035));
        secondaryController.axisLessThan(XboxController.Axis.kLeftY.value, -0.85)
                .onTrue(new NudgeElevatorCommand(mElevatorSubsystem, -0.15));

        // Move Elevator Height
        secondaryController.povDown().onTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eReceive));
        secondaryController.povLeft().onTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL1));
        secondaryController.povUp().onTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL2));
        secondaryController.povRight().onTrue(new MoveToHeightCommand(mElevatorSubsystem, ElevatorPosEnum.eL3));

        secondaryController.leftTrigger().onTrue(new CoralGrappleCommand(mElevatorSubsystem, true));
        secondaryController.rightTrigger().onTrue(new CoralGrappleCommand(mElevatorSubsystem, false));

        secondaryController.a().whileTrue(new PushOutCommand(mAlgaePushSubsystem));
        secondaryController.b().whileTrue(new PushInCommand(mAlgaePushSubsystem));

    }

    public Command getAutonomousCommand() {
        //System.out.println("Setting Autonomous Command: " + sequenceBlue4CoralCommand.getName());
        //return sequenceBlue4CoralCommand;
        //return Auto_001_Blue_Outside_Coral_J2J1I2I1;
        return Auto_002_Blue_Move_Somewhere;
        //return autoChooser.getSelected();  // Use this one to list all the available autopaths in the deploy directory.
    }
}
