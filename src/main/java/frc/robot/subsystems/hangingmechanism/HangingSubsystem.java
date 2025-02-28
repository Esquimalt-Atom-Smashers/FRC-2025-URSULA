package frc.robot.subsystems.hangingmechanism;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangingSubsystem extends SubsystemBase{
    //Spark Max
    private SparkMax winchMotor = new SparkMax(0, MotorType.kBrushless);
    private SparkMaxConfig winchConfig = new SparkMaxConfig();
    private SparkClosedLoopController winchController = winchMotor.getClosedLoopController();
    private RelativeEncoder winchEncoder = winchMotor.getEncoder();

    //Release Servo
    private Servo releaseServo = new Servo(0); //Check value

    public enum ReleaseServoPosition {
        LATCHED(0.5),
        FREE(1);

        double value;

        private ReleaseServoPosition(double value) {
            this.value = value;
        }
    }

    public enum WinchPosition {
        RETRACTED(0),
        EXTENDED(100);

        double value;

        private WinchPosition(double value) {
            this.value = value;
        }
    }

    public HangingSubsystem() {
        winchConfig.encoder.positionConversionFactor(1)
        .velocityConversionFactor(1);

        winchConfig.smartCurrentLimit(1,8,50);

        winchConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1).i(0.00000).d(0.0000)
        .outputRange(-0.5, 0.5, ClosedLoopSlot.kSlot0);


        winchMotor.configure(winchConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        winchController.setReference(0, SparkMax.ControlType.kPosition);
        setReleaseServoPosition(ReleaseServoPosition.FREE);

    }

    public void setReleaseServoPosition(ReleaseServoPosition position) {
        releaseServo.set(position.value);
    }

    public void setWinchPosition(double position) {
        winchController.setReference(position, ControlType.kPosition);
    }

    public void setWinchPosition(WinchPosition position) {
        setWinchPosition(position.value);
    }

    //Getters

    public double getWinchPosition() {
        return winchEncoder.getPosition();
    }

    //Commands

    public SequentialCommandGroup extendHangingMechanismCommand = new SequentialCommandGroup(
        new ReleaseServoCommand(this, ReleaseServoPosition.FREE),
        new WinchToPositionCommand(this, WinchPosition.EXTENDED)
    );

    public SequentialCommandGroup retractHangingMechanismCommand = new SequentialCommandGroup(
        new ReleaseServoCommand(this, ReleaseServoPosition.LATCHED),
        new WinchToPositionCommand(this, WinchPosition.RETRACTED)  
    );

    //Hardware Testing Commands

}
