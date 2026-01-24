// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// WPI Imports
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// CTRE Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;


public class FeederSubsystem extends SubsystemBase {
    private final TalonFXS feederMotor;
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    /** Creates a new IntakeSubsystem. */
    public FeederSubsystem(TalonFXS motor) {
        this.feederMotor = motor;
        configureMotor();
    }

    private void configureMotor() {

        TalonFXSConfiguration cfg = new TalonFXSConfiguration();
        cfg.Commutation.MotorArrangement = MotorArrangementValue.Brushed_DC;
        
        // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));
        
        feederMotor.getConfigurator().apply(cfg);
    }

    public void run(double dutyCycle) {
        SmartDashboard.putNumber("Feeder/ Target Duty Cycle", dutyCycle);
        feederMotor.setControl(dutyRequest.withOutput(dutyCycle));
    }

    public void stop() {
        feederMotor.set(0.0);
    }

    public void setpower(double power) {
        feederMotor.set(power);
    }    

    public double getDutyCycle(){
        return feederMotor.getDutyCycle().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Feeder/ Duty Cycle", getDutyCycle());
    }
}