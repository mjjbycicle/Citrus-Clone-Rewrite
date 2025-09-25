package frc.robot.subsystems.pivot;

import edu.wpi.first.math.util.Units;
import lib.TunableNumber;

public class PivotConstants {
    public static class IDs {
        public static final int LeftPivotID = 1, RightPivotID = 2;
        public static final int PivotEncoderID = 15;
    }

    public static class PIDs {
        public static final TunableNumber pivotKP = new TunableNumber("Pivot Kp");
        public static final TunableNumber pivotKI = new TunableNumber("Pivot Ki");
        public static final TunableNumber pivotKD = new TunableNumber("Pivot Kd");
        public static final TunableNumber pivotVelocity = new TunableNumber("Pivot Velocity");
        public static final TunableNumber pivotAcceleration = new TunableNumber("Pivot Acceleration");

        static {
            pivotKP.setDefault(1);
            pivotKI.setDefault(0.0);
            pivotKD.setDefault(0.0);
            pivotVelocity.setDefault(0.5);
            pivotAcceleration.setDefault(0.5);
        }
    }

    public enum PivotStates {
        IDLE(0.6),
        UP(0.3),
        BACK(0.05);

        public final double position;

        PivotStates(double position) {
            this.position = position;
        }
        
        public static double getSimPosition(double currentPosition) {
            double simUp = Units.degreesToRadians(90);
            double simDown = Units.degreesToRadians(-5);
            return (IDLE.position - currentPosition) / (IDLE.position - UP.position) 
                    * (simUp - simDown) + simDown;
        }
        
        public static double fromSimPosition(double currentPosition) {
            double simUp = Units.degreesToRadians(90);
            double simDown = Units.degreesToRadians(-5);
            return IDLE.position - (currentPosition - simDown) / (simUp - simDown)
                    * (IDLE.position - UP.position);
        }
    }
  
    public static final double gearing = 576;
    public static final double length = Units.inchesToMeters(25);
    public static final double mass = Units.lbsToKilograms(25);
}
