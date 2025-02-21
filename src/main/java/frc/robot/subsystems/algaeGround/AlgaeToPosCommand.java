// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeGround;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlgaeToPosCommand extends Command {
  double positionDegrees =0;  
  private AlgaeGroundSubsystem algaeGroundSubsystem; 
  private boolean atPosition = false;

  public AlgaeToPosCommand(double positionDegrees,AlgaeGroundSubsystem algaeSubsystem) {
    this.positionDegrees = positionDegrees;
    this.algaeGroundSubsystem= algaeSubsystem;
    this.addRequirements(algaeGroundSubsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("StartingAlgaeMove");
    algaeGroundSubsystem.setArmAngle(positionDegrees);
    atPosition = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(algaeGroundSubsystem.algaeArmEncoder.getPosition()-positionDegrees)<1){
      atPosition=true;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPosition;
  }
}
