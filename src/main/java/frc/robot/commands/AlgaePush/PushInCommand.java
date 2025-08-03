package frc.robot.commands.AlgaePush;

import frc.robot.subsystems.AlgaePushSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.PusherConstants;

public class PushInCommand extends Command {
    private final AlgaePushSubsystem mAlgaePush;

    /**
     * Creates a new PushInCommand.
     *
     * @param algaePushSubsystem The AlgaePushSubsystem used by this command.
     */
    public PushInCommand(AlgaePushSubsystem algaePushSubsystem) {
        mAlgaePush = algaePushSubsystem;
        addRequirements(algaePushSubsystem);
    }

    @Override
    public void execute() {
        mAlgaePush.setPushSpeed(0.75);
    }

    @Override
    public void end(boolean interrupted) {
        mAlgaePush.setPushSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(mAlgaePush.getPushPosition()) <= PusherConstants.kTolerance;
    }

}