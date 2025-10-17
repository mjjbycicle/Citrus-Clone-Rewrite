package frc.robot.subsystems.elevator;

import lib.TunableNumber;

import java.util.List;

public class ElevatorConstants {
    public static class IDs {
        public static final int LeftElevatorID = 16, RightElevatorID = 15;
    }

    public static class PIDs {
        public static final TunableNumber elevatorKP = new TunableNumber("Elevator/Elevator Kp");
        public static final TunableNumber elevatorKI = new TunableNumber("Elevator/Elevator Ki");
        public static final TunableNumber elevatorKD = new TunableNumber("Elevator/Elevator Kd");
        public static final TunableNumber elevatorVelocity = new TunableNumber("Elevator/Elevator Velocity");
        public static final TunableNumber elevatorAcceleration = new TunableNumber("Elevator/Elevator Acceleration");
        public static final TunableNumber elevatorKS = new TunableNumber("Elevator/Elevator KS");
        public static final TunableNumber elevatorKV = new TunableNumber("Elevator/Elevator KV");
        public static final TunableNumber elevatorKG = new TunableNumber("Elevator/Elevator KG");

        static {
            elevatorKP.setDefault(0.0);
            elevatorKI.setDefault(0.0);
            elevatorKD.setDefault(0);
            elevatorVelocity.setDefault(0);
            elevatorAcceleration.setDefault(0);
            elevatorKS.setDefault(0);
            elevatorKV.setDefault(0.0);
            elevatorKG.setDefault(0);
        }

        public static final List<TunableNumber> tunableNumbers = List.of(
                elevatorKP,
                elevatorKI,
                elevatorKD,
                elevatorVelocity,
                elevatorAcceleration,
                elevatorKS,
                elevatorKG,
                elevatorKV
        );
    }

    public enum ElevatorStates {
        IDLE(0),
        L4(27),
        L3(22),
        L2(8),
        L1(3.25);

        public final double position;

        ElevatorStates(double position) {
            this.position = position;
        }
    }
}
