package frc.robot.subsystems.algaeremover;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRemoverSubsystem extends SubsystemBase{
    //Wrist Motor
    private SparkMax wristMotor = new SparkMax(0, MotorType.kBrushless);
    private SparkMaxConfig wristConfig = new SparkMaxConfig();
    private SparkClosedLoopController wristController = wristMotor.getClosedLoopController();
    private RelativeEncoder wristEncoder = wristMotor.getEncoder();

    //Spinning Motor
    private SparkMax spinnyMotor = new SparkMax(1, MotorType.kBrushed);
    private SparkMaxConfig spinnyConfig = new SparkMaxConfig();
    private SparkClosedLoopController spinnyController = spinnyMotor.getClosedLoopController();
    private RelativeEncoder spinnyEncoder = spinnyMotor.getEncoder();

    public enum SpinnySpeeds {
        FAST(3000),
        SLOW(1500),
        STOPPED(0),
        REVERSE_FAST(-3000),
        REVERSE_SLOW(-1500);

        double value;

        private SpinnySpeeds(double value) {
            this.value = value;
        }
    }

    public enum WristPositions {
        UP(24.5),
        DOWN(0);

        double value;

        private WristPositions(double value) {
            this.value = value;
        }
    }

    public AlgaeRemoverSubsystem() {
        setupMotors();
    }

    private void setupMotors() {
        //Wrist Motor
        wristConfig.encoder.positionConversionFactor(1/1.785)
        .velocityConversionFactor(1);

        wristConfig.smartCurrentLimit(1,8,50);

        wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1).i(0.000001).d(0.0000)
        .outputRange(-0.5, 0.6, ClosedLoopSlot.kSlot0);

        wristConfig.closedLoop.maxMotion
        .maxVelocity(3000)
        .maxAcceleration(8000)
        .allowedClosedLoopError(1).positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        wristController.setReference(0, SparkMax.ControlType.kPosition);

        //Spinning Motor
        
        spinnyConfig.encoder.positionConversionFactor(1/1.785)
        .velocityConversionFactor(1);

        spinnyConfig.smartCurrentLimit(1,8,50);

        spinnyConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0001).i(0).d(0).velocityFF(1.0 / 5767)
        .outputRange(-0.6, 0.6, ClosedLoopSlot.kSlot0);

        spinnyConfig.closedLoop.maxMotion
        .maxVelocity(3000)
        .maxAcceleration(8000)
        .allowedClosedLoopError(1);

        spinnyMotor.configure(spinnyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setWristPosition(WristPositions position) {
        setWristPosition(position.value);
    }

    public void setWristPosition(double position) {
        wristController.setReference(position, ControlType.kPosition);
    }

    public void setSpinnySpeed(double speed) {
        spinnyController.setReference(speed, ControlType.kVelocity);
    }

    public void setSpinnySpeed(SpinnySpeeds speed) {
        setSpinnySpeed(speed.value);
    }

    public double getWristPosition() {
        return wristEncoder.getPosition();
    }

    public double getSpinnySpeed() {
        return spinnyEncoder.getVelocity();
    }
}