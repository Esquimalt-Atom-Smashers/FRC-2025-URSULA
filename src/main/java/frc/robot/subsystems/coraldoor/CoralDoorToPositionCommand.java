package frc.robot.subsystems.coraldoor;

import edu.wpi.first.wpilibj2.command.Command;

public class CoralDoorToPositionCommand extends Command{
    CoralDoorSubsystem coralDoorSubsystem;
    CoralDoorSubsystem.DoorPosition targetPosition;

    public CoralDoorToPositionCommand(CoralDoorSubsystem.DoorPosition targetPosition, CoralDoorSubsystem coralDoorSubsystem) {
        this.coralDoorSubsystem = coralDoorSubsystem;
        this.targetPosition = targetPosition;
    }

    @Override
    public void initialize() {
        coralDoorSubsystem.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
