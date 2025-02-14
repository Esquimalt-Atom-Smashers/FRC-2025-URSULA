package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ElevatorSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static final double lowPosition=0;
  public static final double processorPosition=10;
  public static final double level1Position=20;
  public static final double level2Position=30;
  public static final double level3Position=40;
  public static final double level4Position=50;
  public static final double netPosition=60;
  public static final double coralStationPosition=25;



  //add a timer object
  private Timer timer = new Timer();

  //add a new spark controller
  //private Spark spark = new Spark(0);
  //add anew sparkmax brushless
  private SparkMax elevatorMotor = new SparkMax(1, MotorType.kBrushless);
  //add new victor spx
  //private WPI_VictorSPX victor = new WPI_VictorSPX(0);
  private SparkMaxConfig elevatorConfig=new SparkMaxConfig();
  private SparkClosedLoopController elevatorPIDController=elevatorMotor.getClosedLoopController();
  public RelativeEncoder elevatorEncoder=elevatorMotor.getEncoder();
  

 
  public ElevatorSubsystem() {
    // Initialize the subsystem here
    timer.start();
    elevatorConfig.encoder.positionConversionFactor(1)
    .velocityConversionFactor(1);

    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1).i(0.0001).d(0.01);
    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorPIDController.setReference(0, SparkMax.ControlType.kPosition);    
  }

  

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if(timer.hasElapsed(2.0)) {
      System.out.println("Elevator Running");
      timer.reset();
    }
  }
  public void setTargetPosition(double targetPosition){
    elevatorPIDController.setReference(targetPosition, ControlType.kPosition);

  }

}
