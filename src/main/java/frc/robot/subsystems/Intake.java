package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import static frc.robot.Constants.IntakeConstants.*;
import frc.robot.mechanics.GearRatio;

public class Intake extends SubsystemBase {

    private final TalonFXS intake = new TalonFXS(INTAKE_MOTOR_PORT);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    public Intake() {

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
        intake.getConfigurator().apply(config);
    }

    /** Run intake in open-loop (percent output) */
    public void runIntake(double power) {
        intake.set(power);
    }

    /** Stop  intake */
    public void stopIntake() {
        intake.set(0);
    }

    /** Run intake in closed-loop velocity mode (RPM) */
    public void setFXSRPM(double rpm) {
        //double rps = rpm / 60.0;
        GearRatio gearRatio = new GearRatio(2, 1);
        double rps = gearRatio.outputToMotor(rpm / 60.0);
        intake.setControl(velocityRequest.withVelocity(rps));
    }

    public double getFXSRPM() {
        double  intakeRPS = intake.getVelocity().getValueAsDouble();
        GearRatio gearRatio = new GearRatio(2, 1);
        double wheelRPS = gearRatio.motorToOutput(intakeRPS);
        return wheelRPS * 60.0;
    }


}
