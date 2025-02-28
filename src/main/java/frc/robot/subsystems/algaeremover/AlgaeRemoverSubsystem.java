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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRemoverSubsystem extends SubsystemBase{
    //Wrist Motor
    private SparkMax wristMotor = new SparkMax(5, MotorType.kBrushless);
    private SparkMaxConfig wristConfig = new SparkMaxConfig();
    private SparkClosedLoopController wristController = wristMotor.getClosedLoopController();
    private RelativeEncoder wristEncoder = wristMotor.getEncoder();

    //Spinning Motor
    private SparkMax spinnyMotor = new SparkMax(6, MotorType.kBrushed); //no encoder for spinny motor
    private SparkMaxConfig spinnyConfig = new SparkMaxConfig();
   
    public enum SpinnyPowers {
        UPPER(4),//Volts
        LOWER(-4),//Volts TODO check if this is the right direction
        STOPPED(0);

        double value;

        private SpinnyPowers(double value) {
            this.value = value;
        }
    }

    public enum WristPositions {
        UP(0.5),//half a rotation
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
        wristConfig.encoder.positionConversionFactor(1/49);

        wristConfig.smartCurrentLimit(2);

        wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1).i(0).d(0)
        .outputRange(-0.5, 0.6, ClosedLoopSlot.kSlot0);

        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        wristController.setReference(0, SparkMax.ControlType.kPosition);

        //Spinning Motor
        spinnyConfig.smartCurrentLimit(4); 
        spinnyMotor.configure(spinnyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        spinnyMotor.setVoltage(0);
    }

    public void setWristPosition(WristPositions position) {
        setWristPosition(position.value);
    }

    public void setWristPosition(double position) {
        wristController.setReference(position, ControlType.kPosition);
    }

    public void setSpinnyPower(double speed) {
        spinnyMotor.setVoltage(speed);
    }

    public void setSpinnyPower(SpinnyPowers speed) {
        setSpinnyPower(speed.value);
    }

    public double getWristPosition() {
        return wristEncoder.getPosition();
    }

    public SequentialCommandGroup removeUpperAlgae = new SequentialCommandGroup(
        new SpinAlgaeSpinnyThing(this, SpinnyPowers.UPPER),
        new MoveAlgaeWristCommand(this, WristPositions.UP),
        new MoveAlgaeWristCommand(this, WristPositions.DOWN),
        new SpinAlgaeSpinnyThing(this, SpinnyPowers.STOPPED)
    );

    public SequentialCommandGroup removeLowerAlgae = new SequentialCommandGroup(
        new SpinAlgaeSpinnyThing(this, SpinnyPowers.LOWER),
        new MoveAlgaeWristCommand(this, WristPositions.DOWN),
        new SpinAlgaeSpinnyThing(this, SpinnyPowers.STOPPED)
    );
}