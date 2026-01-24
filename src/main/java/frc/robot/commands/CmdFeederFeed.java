// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;

//Import subsystems
import frc.robot.subsystems.FeederSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdFeederFeed extends Command {
  /** Creates a new CmdFeederFeed. */

  public final FeederSubsystem feeder;
  public double dutyCycle;
  
  public CmdFeederFeed(FeederSubsystem feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feeder = feeder;
  

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.run(dutyCycle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void updateDC(double DC)
  {
    this.dutyCycle = DC;
  }

}
