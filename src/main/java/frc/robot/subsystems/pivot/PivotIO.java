package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    class PivotIOInputs {
        public double absoluteEncoderPositionRotations = 0.0;
        public double velocityRotationsPerSecond = 0.0;
        public double[] appliedVolts = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] tempCelcius = new double[] {};
        
        public boolean absoluteEncoderConnected = true;
    }
    
    default void updateInputs(PivotIOInputs inputs) {}
    
    default void runVolts(double volts) {}
    
    default void setBrakeMode(boolean enabled) {}
    
    default void stop() {}
}
