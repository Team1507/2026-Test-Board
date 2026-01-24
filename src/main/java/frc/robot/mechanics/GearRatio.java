package frc.robot.mechanics;

public class GearRatio {
    private final double ratio ;

    public GearRatio(double input, double output) { 
        this.ratio = input/output;
    }

    /** Convert motor rotations to output rotations */
    public double motorToOutput(double motorRotations) {
        return motorRotations / ratio;
    }

    /** Convert output rotations to motor rotations */
    public double outputToMotor(double outputRotations) {
        return outputRotations * ratio;
    }
    
}
