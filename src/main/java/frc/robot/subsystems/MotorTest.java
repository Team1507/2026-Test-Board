package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import static frc.robot.Constants.MotorTest.*;

public class MotorTest extends SubsystemBase {

    private final TalonFXS motor = new TalonFXS(MOTOR_PORT);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    public MotorTest() {

        TalonFXSConfiguration config = new TalonFXSConfiguration();

        // --- REQUIRED FOR MINION MODE ---
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // --- PID + FEEDFORWARD GAINS ---
        config.Slot0.kS = 0.1;
        config.Slot0.kV = 0.12;
        config.Slot0.kP = 0.11;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        // --- VOLTAGE LIMITS ---
        config.Voltage.withPeakForwardVoltage(Volts.of(8))
                      .withPeakReverseVoltage(Volts.of(-8));

        // Apply configuration
        motor.getConfigurator().apply(config);
    }

    /** Run motor in open-loop (percent output) */
    public void runMotor(double power) {
        motor.set(power);
    }

    /** Stop motor */
    public void stopMotor() {
        motor.set(0);
    }

    /** Run motor in closed-loop velocity mode (RPM) */
    public void setRPM(double rpm) {
        double rps = rpm / 60.0;
        motor.setControl(velocityRequest.withVelocity(rps));
    }
}
