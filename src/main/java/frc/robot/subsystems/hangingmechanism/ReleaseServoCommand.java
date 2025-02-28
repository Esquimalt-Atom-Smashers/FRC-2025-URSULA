package frc.robot.subsystems.hangingmechanism;

import edu.wpi.first.wpilibj2.command.Command;

public class ReleaseServoCommand extends Command{
    private HangingSubsystem hangingSubsystem;
    private HangingSubsystem.ReleaseServoPosition targetPosition;

    public ReleaseServoCommand(HangingSubsystem hangingSubsystem, HangingSubsystem.ReleaseServoPosition targetPosition) {
        this.hangingSubsystem = hangingSubsystem;
        this.targetPosition = targetPosition;

        this.addRequirements(hangingSubsystem);
    }

    @Override
    public void initialize() {
        hangingSubsystem.setReleaseServoPosition(targetPosition);
    }
}
