// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// WPI Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

// CTRE Libraries
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.*;

// Constants
import static frc.robot.Constants.MotorTest.*;

public class MotorTest extends SubsystemBase {

  private final CANBus canbus = new CANBus("canivore");
  private final TalonFX m_fxs = new TalonFX(MOTOR_PORT, canbus);
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  /** Creates a new MotorTest. */
  public MotorTest() { 
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* 
     * Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor 
     */
    configs.Slot0.kS = 0.1;   // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12;  // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.11;  // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0;     // No output for integrated error
    configs.Slot0.kD = 0;     // No output for error derivative

    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    // Apply Talon Configuration to the motor
    m_fxs.getConfigurator().apply(configs);
  }

  public void runMotor(double power) {
    m_fxs.set(power);
  }

  public void stopMotor() {
    m_fxs.set(0);
  }

  @Override
    public void periodic() {
    // This method will be called once per scheduler run
  }
}
