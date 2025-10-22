package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.enums.ElevatorPosEnum;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveToHeightCommand extends Command {
    private final ElevatorSubsystem mElevator;
    private final ElevatorPosEnum mTargetHeightEnum;

    /**
     * Creates a new MoveToHeightCommand
     * 
     * @param elevatorSubsystem The subsystem used by this command
     */
    public MoveToHeightCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPosEnum targetHeightEnum) {
        mElevator = elevatorSubsystem;
        mTargetHeightEnum = targetHeightEnum;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        mElevator.setDesiredElevatorState(mTargetHeightEnum);
    }

    @Override
    public boolean isFinished() {
        return mElevator.getElevatorState() == mTargetHeightEnum;
    }
}
