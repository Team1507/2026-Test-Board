// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Constants
import static frc.robot.Constants.OperatorConstants.DRIVE_CONTROLLER_PORT;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// WPI Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// Commands
import frc.robot.commands.CmdMotorRunSubway;
import frc.robot.commands.CmdRunShooter;
import frc.robot.commands.CmdShooterPIDTuner;
// Subsystems
import frc.robot.subsystems.MotorTest;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.Shooter.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public MotorTest m_motortest = new MotorTest();
  double shooterTargetRPM = 25;


  public final ShooterSubsystem shooterSubsystem =
    new ShooterSubsystem(
        new TalonFX(SHOOTER_CAN_ID)
    );

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
    new CommandXboxController(DRIVE_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driverController.a().whileTrue(new CmdMotorRunSubway(m_motortest));

    // m_driverController.a().onTrue(new CmdRunShooter(shooterSubsystem, shooterTargetRPM));
    // m_driverController.b().onTrue(new CmdRunShooter(shooterSubsystem, 0.0));
    //m_driverController.a().whileTrue(new CmdMotorRunSubway, 0.2);


    SmartDashboard.putData( 
        "Run Shooter PID Tuner",
        new CmdShooterPIDTuner(shooterSubsystem, shooterTargetRPM) // max RPM here
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
