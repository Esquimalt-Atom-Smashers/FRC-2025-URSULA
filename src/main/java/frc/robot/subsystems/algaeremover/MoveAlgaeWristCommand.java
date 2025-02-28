package frc.robot.subsystems.algaeremover;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveAlgaeWristCommand extends Command{
    private AlgaeRemoverSubsystem algaeRemoverSubsystem;
    private double targetPosition;

    public MoveAlgaeWristCommand(AlgaeRemoverSubsystem algaeRemoverSubsystem, double targetPosition) {
        this.algaeRemoverSubsystem = algaeRemoverSubsystem;
        this.targetPosition = targetPosition;
    }

    public MoveAlgaeWristCommand(AlgaeRemoverSubsystem algaeRemoverSubsystem, AlgaeRemoverSubsystem.WristPositions targetPosition) {
        this.algaeRemoverSubsystem = algaeRemoverSubsystem;
        this.targetPosition = targetPosition.value;
    }

    @Override
    public void initialize() {
        algaeRemoverSubsystem.setWristPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {        
        return Math.abs(algaeRemoverSubsystem.getWristPosition() - targetPosition) < 1.0;
    }
}
