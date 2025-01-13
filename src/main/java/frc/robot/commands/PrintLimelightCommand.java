// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.limelight.LimelightSubsystem;

/** An example command that uses an example subsystem. */
public class PrintLimelightCommand extends Command {
  private String string;
  private LimelightSubsystem limelightSubsystem;
  
   

  public PrintLimelightCommand(String string) {
    this.string = string;
  }
  public PrintLimelightCommand(String string, LimelightSubsystem limelightSubsystem) {
    this.string = string;
    this.limelightSubsystem = limelightSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(string);
    if(string.equals("XYZ")){
      double []values = limelightSubsystem.getTargets();
      System.out.print(values[0]+"; ");
      System.out.print(values[1]+"; ");
      System.out.println(values[2]+"; ");

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
