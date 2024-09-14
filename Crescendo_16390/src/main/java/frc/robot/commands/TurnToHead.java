// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;

public class TurnToHead extends Command {
  PIDController pid = new PIDController(0.008, 0, 0);
  Drivetrain6390 drivetrain;
  double rot;
  /** Creates a new TurnToHead. */
  public TurnToHead(Drivetrain6390 drivetrain, double rot) {
    this.drivetrain = drivetrain;
    this.rot = rot;
    // addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    pid.setSetpoint(rot);
    pid.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
  
      drivetrain.feedbackDrive(new ChassisSpeeds(0,0, pid.calculate(drivetrain.getHeading())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.feedbackDrive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
