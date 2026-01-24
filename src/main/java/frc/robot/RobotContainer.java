// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Constants
import static frc.robot.Constants.OperatorConstants.DRIVE_CONTROLLER_PORT;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// WPI Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// Commands
import frc.robot.commands.CmdMotorRunSubway;
import frc.robot.commands.CmdIntakeRun;
import frc.robot.commands.CmdKrackRunSubway;
import frc.robot.commands.CmdRunShooter;
import frc.robot.commands.CmdShooterPIDTuner;
import frc.robot.commands.CmdFeederFeed;
// Subsystems
import frc.robot.subsystems.MotorTest;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Feeder.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public MotorTest m_motortest = new MotorTest();
  public Intake m_intake = new Intake();
  double shooterTargetRPM = 25;

  double feederTargetDC = 0.0;


  public final ShooterSubsystem shooterSubsystem =
    new ShooterSubsystem(
        new TalonFX(SHOOTER_CAN_ID)
    );

  public final FeederSubsystem feederSubsystem = 
    new FeederSubsystem(
      new TalonFXS(FEEDER_CAN_ID)
    );

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
    new CommandXboxController(DRIVE_CONTROLLER_PORT);


  private CmdFeederFeed m_FeederFeed = new CmdFeederFeed(feederSubsystem); 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureShooterDefault();
  }

  private void configureShooterDefault() {

        shooterSubsystem.setDefaultCommand(
            Commands.run(
              () -> {
                // Build telemetry → ask model → set RPM
                //shooterSubsystem.updateShooterFromModel();
                shooterSubsystem.setTargetRPM(shooterTargetRPM);
              },
              shooterSubsystem
            )
        );

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
    m_driverController.b().whileTrue(new CmdIntakeRun(m_intake));
    m_driverController.x().toggleOnTrue(m_FeederFeed);

    // m_driverController.a().onTrue(new CmdRunShooter(shooterSubsystem, shooterTargetRPM));
    // m_driverController.b().onTrue(new CmdRunShooter(shooterSubsystem, 0.0));
    //m_driverController.a().whileTrue(new CmdMotorRunSubway, 0.2);
    SmartDashboard.putNumber("Feeder Dutycycle", feederTargetDC);
    SmartDashboard.putNumber("ShooterTargetRPM", shooterTargetRPM);

    // SmartDashboard.putData( 
    //     "Run FeederFeed",
    //     new CmdFeederFeed(feederSubsystem, feederTargetDC) // max RPM here
    // );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void updateSD()
  {
    feederTargetDC = SmartDashboard.getNumber("Feeder Dutycycle", feederTargetDC);
    shooterTargetRPM = SmartDashboard.getNumber("ShooterTargetRPM", shooterTargetRPM);
    m_FeederFeed.updateDC(feederTargetDC);
  }


}
