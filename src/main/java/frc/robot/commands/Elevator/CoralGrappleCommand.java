package frc.robot.commands.Elevator;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralGrappleCommand extends Command {
    private final ElevatorSubsystem mElevator;
    private final boolean isInput;

    /**
     * Creates a new CoralGrappleCommand.
     * 
     * @param elevator The ElevatorSubsystem used by this command.
     * @param isInput True if the command should input coral, false to output.
     */
    public CoralGrappleCommand(ElevatorSubsystem elevatorSubsystem, boolean isInput) {
      this.mElevator = elevatorSubsystem;
      this.isInput = isInput;
      addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        mElevator.setGrappleSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        mElevator.setGrappleSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        if (isInput) {
            return mElevator.isCoralIn();
        }
        return !mElevator.isCoralIn();
    }
}
