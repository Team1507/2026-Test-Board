// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /*
   * This nested class is used for Operator Constants such as:
   *  - Controller Ports
   */
  public static class OperatorConstants {
    public static final int DRIVE_CONTROLLER_PORT = 0;
  }

  /*
   * Class is used for Constants Related to the Motor Test Subsystem
   */
  public static class MotorTest {
    public static final int MOTOR_PORT = 5;
  }

  public static class Shooter {
    // ============================================================
    // Hardware
    // ============================================================
    public static final int SHOOTER_CAN_ID = 1;

    // Maximum wheel RPM (for UI, clamping, etc.)
    public static final double MAX_RPM = 2000.0;

    // ============================================================
    // Control Gains (Phoenix Slot0)
    // ============================================================
    public static final class Gains {
        // PID
        public static final double KP = 0.013;  // 0.013
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        // Feedforward
        public static final double KV = 0.1353;  // volts per motor RPS  0.1353
        public static final double KS = 0.0;
        public static final double KA = 0.0;
    }

    // ============================================================
    // Flywheel Physics Constants
    // ============================================================
    public static final class Flywheel {

        // Wheel inertia (kg·m²)
        public static final double INERTIA = 0.00045;

        // Motor Kv: rad/s per volt
        // Falcon free speed: 5330 RPM @ 12V
        public static final double MOTOR_KV =
            (5330.0 * (2.0 * Math.PI / 60.0)) / 12.0;

        // Motor torque constant (N·m per amp)
        public static final double MOTOR_KT = 0.018;

        // Motor winding resistance (ohms)
        public static final double MOTOR_RESISTANCE = 0.09;

        // Static friction torque (N·m)
        public static final double FRICTION_TORQUE = 0.002;
    }

    // ============================================================
    // Simulation Behavior (Phoenix‑style smoothing)
    // ============================================================
    public static final class Sim {

        // Sensor velocity filtering (seconds)
        // Phoenix applies ~10–40 ms smoothing internally
        public static final double SENSOR_FILTER_TIME_CONSTANT = 0.04;

        // Commanded velocity filtering (seconds)
        // Phoenix smooths target velocity changes
        public static final double COMMAND_FILTER_TIME_CONSTANT = 0.08;

        // Voltage slew rate (V/s)
        // Phoenix ramps voltage internally to avoid instant jumps
        public static final double VOLTAGE_SLEW_RATE = 24.0;

        // Max acceleration clamp (RPM/s)
        // Helps prevent unrealistic physics spikes
        public static final double MAX_ACCEL_RPM_PER_SEC = 8000.0;

        // Max battery voltage
        public static final double MAX_VOLTAGE = 12.0;
    }

  }
}
