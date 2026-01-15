// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// WPI Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Subsystems
import frc.robot.subsystems.MotorTest;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdMotorRunSubway extends Command {
  
  public final MotorTest motorTest;

  public CmdMotorRunSubway(MotorTest motorTest) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motorTest = motorTest;

    addRequirements(motorTest);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorTest.runMotor(.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorTest.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
