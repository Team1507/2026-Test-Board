package frc.robot.subsystems;

// WPILib
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

// CTRE Phoenix 6
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import static frc.robot.Constants.MotorTest.*;

public class MotorTest extends SubsystemBase {

  //private final CANBus canbus = new CANBus("canivore");
  private final TalonFXS motor = new TalonFXS(MOTOR_PORT);
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  public MotorTest() {
    // Create default hardware config
    TalonFXSConfiguration config = new TalonFXSConfiguration();

    //config = new TalonFXSConfiguration();

    //config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    motor.getConfigurator().apply(config);

    // Configure PID/FF gains
    config.Slot0.kS = 0.1;
    config.Slot0.kV = 0.12;
    config.Slot0.kP = 0.11;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    // Voltage limits
    config.Voltage.withPeakForwardVoltage(Volts.of(8))
                  .withPeakReverseVoltage(Volts.of(-8));

    // Apply config to the motor controller
    motor.getConfigurator().apply(config);
  }

  public void runMotor(double power) {
    motor.set(power);
  }

  public void stopMotor() {
    motor.set(0);
  }
}
