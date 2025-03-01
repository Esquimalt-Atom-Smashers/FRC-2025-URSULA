package frc.robot.subsystems.coraldoor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class CoralDoorSubsystem extends SubsystemBase{
    
    //Hardware Components
    private Servo coralServo;

    public enum DoorPosition {
        OPEN(1),
        CLOSED(0); //Check these values later
        
        double value;

        private DoorPosition(double value) {
            this.value = value;
        }
    }

    private boolean positionChanged = true;
    private DoorPosition doorPosition = DoorPosition.CLOSED;

    public CoralDoorSubsystem() {
        coralServo = new Servo(0); //Check this value later
    }

    public void setPosition(DoorPosition position) {
        coralServo.set(position.value);
        positionChanged = true;
    }

    @Override
    public void periodic() {
        if(positionChanged) {
            System.out.println("Coral Door Position: " + ((doorPosition == DoorPosition.OPEN) ? "OPEN" : "CLOSED"));
            positionChanged = false;
        }
    }
}