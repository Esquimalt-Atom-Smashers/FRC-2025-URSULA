package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ElevatorSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static final double lowPosition=0;
  public static final double processorPosition=-10;
  public static final double level1Position=-20;
  public static final double level2Position=-30;
  public static final double level3Position=-40;
  public static final double level4Position=-50;
  public static final double netPosition=-60;
  public static final double coralStationPosition=-70;



  //add a timer object
  private Timer timer = new Timer();

  //add a new spark controller
  //private Spark spark = new Spark(0);
  //add anew sparkmax brushless
  protected SparkMax elevatorMotor = new SparkMax(2, MotorType.kBrushless);
  //add new victor spx
  //private WPI_VictorSPX victor = new WPI_VictorSPX(0);
  protected SparkMaxConfig elevatorConfig=new SparkMaxConfig();
  protected SparkClosedLoopController elevatorClosedLoopController=elevatorMotor.getClosedLoopController();
  public RelativeEncoder elevatorEncoder=elevatorMotor.getEncoder();
  

 
  public ElevatorSubsystem() {
    // Initialize the subsystem here
    timer.start();
    elevatorConfig.encoder.positionConversionFactor(1/1.785)
    .velocityConversionFactor(1);
    elevatorConfig.smartCurrentLimit(1,8,50);

    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1).i(0.000001).d(0.0000)
    .outputRange(-.4, .6, ClosedLoopSlot.kSlot0);
    // Set PID values for velocity control in slot 1
        // .p(0.0001, ClosedLoopSlot.kSlot1)
        // .i(0, ClosedLoopSlot.kSlot1)
        // .d(0, ClosedLoopSlot.kSlot1)
        // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        // .outputRange(-.4, .6, ClosedLoopSlot.kSlot1);

    elevatorConfig.closedLoop.maxMotion
    // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(3000)
        .maxAcceleration(8000)
        .allowedClosedLoopError(1).positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        


    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorClosedLoopController.setReference(0, SparkMax.ControlType.kPosition);    
  }

  

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if(timer.hasElapsed(2.0)) {
      System.out.println("Elevator Running at position"+elevatorEncoder.getPosition());
      timer.reset();
    }
  }
  public void setTargetPosition(double targetPosition){
    elevatorClosedLoopController.setReference(targetPosition, ControlType.kPosition);
    
    //elevatorClosedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl,
    //ClosedLoopSlot.kSlot0);


  }
  

}
