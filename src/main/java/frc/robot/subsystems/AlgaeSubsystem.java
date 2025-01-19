package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class AlgaeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //add a timer object
  private Timer timer = new Timer();

  //add a new spark controller
  //private Spark spark = new Spark(0);
  //add anew sparkmax brushless
  //private SparkMax sparkmax = new SparkMax(3, MotorType.kBrushless);
  //add new victor spx
  //private WPI_VictorSPX victor = new WPI_VictorSPX(0);
  
  private SparkMax rollerMotor = new SparkMax(1, MotorType.kBrushless);
  private SparkMax elbowMotor = new SparkMax(4, MotorType.kBrushless);

    private SparkMaxConfig rollerConfig=new SparkMaxConfig();
    private SparkMaxConfig elbowConfig=new SparkMaxConfig();
  private SparkClosedLoopController rollerPIDController=rollerMotor.getClosedLoopController();
  private SparkClosedLoopController elbowPIDController=elbowMotor.getClosedLoopController();
  public RelativeEncoder rollerEncoder=rollerMotor.getEncoder();
  public RelativeEncoder elbowEncoder=elbowMotor.getEncoder();

  public static final double INTAKE_POSITION =50;
  public static final double DRIVE_HOLD_POSITION =-0;
  public static final double PROCESSOR_HOLD_POSITION =0;

  public static final double INTAKE_VOLTAGE =-12;
  public static final double OUTTAKE_VOLTAGE =5;

  

 
  public AlgaeSubsystem() {
    // Initialize the subsystem here
    timer.start();
    rollerConfig.encoder.positionConversionFactor(1)
    .velocityConversionFactor(1);
    rollerConfig.smartCurrentLimit(2,10,300);
    rollerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.001).i(0.0000).d(0.0001);

    elbowConfig.encoder.positionConversionFactor(1)
    .velocityConversionFactor(1);
    elbowConfig.smartCurrentLimit(2,3);

    elbowConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1).i(0.0000).d(0.001);

    
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elbowMotor.configure(elbowConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rollerPIDController.setReference(0, SparkMax.ControlType.kVoltage);
    
    elbowEncoder.setPosition(0);
    elbowPIDController.setReference(0, SparkMax.ControlType.kPosition);
  }
 
  

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if(timer.hasElapsed(1.0)) {
      System.out.println("Hello World");
      timer.reset();
    }
  }
  public Command setSpeedCommand(double targetSpeed){
    return runOnce(()->{rollerPIDController.setReference(targetSpeed, ControlType.kVoltage);});
  }
  public Command setPositionCommand(double targetPos){
    return runOnce(()->{rollerPIDController.setReference(targetPos, ControlType.kPosition);});
  }
  public Command intakeCommand(){
    return runOnce(()->{
      elbowPIDController.setReference(INTAKE_POSITION, SparkMax.ControlType.kPosition);
      rollerPIDController.setReference(INTAKE_VOLTAGE, ControlType.kVoltage);
    });
  }
  public Command holdDriveCommand(){
    return runOnce(()->{
      elbowPIDController.setReference(DRIVE_HOLD_POSITION, SparkMax.ControlType.kPosition);
      rollerPIDController.setReference(INTAKE_VOLTAGE/25, ControlType.kVoltage);
    });
  }
  public Command holdProcessorCommand(){
    return runOnce(()->{
      elbowPIDController.setReference(PROCESSOR_HOLD_POSITION, SparkMax.ControlType.kPosition);
      rollerPIDController.setReference(INTAKE_VOLTAGE/25, ControlType.kVoltage);
    });
  }
  public Command scoreProcessorCommand(){
    return runOnce(()->{
      elbowPIDController.setReference(PROCESSOR_HOLD_POSITION, SparkMax.ControlType.kPosition);
      rollerPIDController.setReference(OUTTAKE_VOLTAGE, ControlType.kVoltage);
    });
  }

}
