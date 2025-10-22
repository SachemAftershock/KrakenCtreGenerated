package frc.robot.commands.Elevator;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class NudgeElevatorCommand extends Command {
    private final ElevatorSubsystem mElevator;
    private double mNudgeSpeed;

    /**
     * Creates a new NudgeElevatorHeightCommand.
     *
     * @param elevatorSubsystem The elevator subsystem.
     * @param nudgeSpeed Applied speed to the motor.
     */
    public NudgeElevatorCommand(ElevatorSubsystem elevatorSubsystem, double nudgeSpeed) {
        mElevator = elevatorSubsystem;
        mNudgeSpeed = nudgeSpeed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        mElevator.setElevatorSpeed(mNudgeSpeed);
    }

    @Override
    public boolean isFinished() {
        Timer.delay(0.25);
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        mElevator.setElevatorSpeed(0);
    }
}