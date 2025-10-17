package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import lib.TunableNumber;

import java.util.function.DoubleSupplier;

public class PivotConstants {
    public static class IDs {
        public static final int LeftPivotID = 1, RightPivotID = 2;
        public static final int PivotEncoderID = 15;
    }

    public static class PIDs {
        public static final TunableNumber pivotKP = new TunableNumber("Pivot/Pivot Kp");
        public static final TunableNumber pivotKI = new TunableNumber("Pivot/Pivot Ki");
        public static final TunableNumber pivotKD = new TunableNumber("Pivot/Pivot Kd");
        public static final TunableNumber pivotVelocity = new TunableNumber("Pivot/Pivot Velocity");
        public static final TunableNumber pivotAcceleration = new TunableNumber("Pivot/Pivot Acceleration");
        public static final TunableNumber pivotKS = new TunableNumber("Pivot/Pivot KS");
        public static final TunableNumber pivotKV = new TunableNumber("Pivot/Pivot KV");
        public static final TunableNumber pivotKG = new TunableNumber("Pivot/Pivot KG");

        static {
            pivotKP.setDefault(22);
            pivotKI.setDefault(0.0);
            pivotKD.setDefault(0.5);
            pivotVelocity.setDefault(10);
            pivotAcceleration.setDefault(3.5);
            pivotKS.setDefault(0.01);
            pivotKV.setDefault(0.0);
            pivotKG.setDefault(-0.01);
        }
    }

    public enum PivotStates {
        IDLE(0.536),
        UP(0.286),
        BACK(0.03),
        UP_FORWARD(0.411),
        UP_BACK(0.161),
        JOYSTICK_LISTEN(0.536);

        public final double position;
        public DoubleSupplier customPositionSupplier = () -> 0.536;

        PivotStates(double position) {
            this.position = position;
        }

        public static double getSimPosition(double currentPosition) {
            double simUp = Units.degreesToRadians(90);
            double simDown = Units.degreesToRadians(0);
            return (IDLE.position - currentPosition) / (IDLE.position - UP.position)
                    * (simUp - simDown) + simDown;
        }

        public static double fromSimPosition(double currentPosition) {
            double simUp = Units.degreesToRadians(90);
            double simDown = Units.degreesToRadians(0);
            return IDLE.position - (currentPosition - simDown) / (simUp - simDown)
                    * (IDLE.position - UP.position);
        }

        public void setPositionSupplier(DoubleSupplier positionSupplier) {
            this.customPositionSupplier = () -> {
                double angle = positionSupplier.getAsDouble();
                if (angle > IDLE.position) return IDLE.position;
                return Math.max(angle, BACK.position);
            };
        }
    }
}
