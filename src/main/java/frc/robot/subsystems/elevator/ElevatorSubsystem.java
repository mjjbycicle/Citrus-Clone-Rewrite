package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.TunableNumber;

import static frc.robot.subsystems.elevator.ElevatorConstants.IDs;
import static frc.robot.subsystems.elevator.ElevatorConstants.PIDs;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevator;
    private final TalonFX follower;
    private final double startPosition;

    private ElevatorConstants.ElevatorStates elevatorState = ElevatorConstants.ElevatorStates.IDLE;

    public ElevatorSubsystem() {
        follower = new TalonFX(IDs.LeftElevatorID);
        elevator = new TalonFX(IDs.RightElevatorID);
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(40)
                ).withFeedback(
                        new FeedbackConfigs()
                                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                ).withSlot0(
                        new Slot0Configs()
                                .withKP(PIDs.elevatorKP.getAsDouble())
                                .withKI(PIDs.elevatorKI.getAsDouble())
                                .withKD(PIDs.elevatorKD.getAsDouble())
                                .withKS(PIDs.elevatorKS.getAsDouble())
                                .withKG(PIDs.elevatorKG.getAsDouble())
                                .withKV(PIDs.elevatorKV.getAsDouble())
                                .withGravityType(GravityTypeValue.Elevator_Static)
                ).withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(PIDs.elevatorAcceleration.getAsDouble())
                                .withMotionMagicCruiseVelocity(PIDs.elevatorVelocity.getAsDouble())
                ).withMotorOutput(
                        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                );
        elevator.getConfigurator().apply(elevatorConfig);
        follower.setControl(new Follower(elevator.getDeviceID(), true));

        startPosition = elevator.getPosition().getValueAsDouble();
    }

    public Command runCurrentState() {
        return this.runOnce(() -> elevator.setControl(new MotionMagicVoltage(elevatorState.position)));
    }

    public Command setState(ElevatorConstants.ElevatorStates newState) {
        return this.runOnce(() -> this.elevatorState = newState);
    }

    public Command setThenRunState(ElevatorConstants.ElevatorStates newState) {
        return setState(newState).andThen(runCurrentState());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Motor Output", elevator.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Motor Output", follower.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Target Position", elevatorState.position);
        SmartDashboard.putNumber("Elevator Position", getPosition());
        boolean shouldRefresh = false;
        for (TunableNumber number : PIDs.tunableNumbers) {
            if (number.hasChanged()) {
                shouldRefresh = true;
                break;
            }
        }
        if (shouldRefresh) {
            refreshGains();
        }
        SmartDashboard.putBoolean("Elevator refresh", shouldRefresh);
    }

    public double getPosition() {
        return elevator.getPosition().getValueAsDouble() - startPosition;
    }

    public void refreshGains() {
        TalonFXConfiguration config = new TalonFXConfiguration()
                .withSlot0(
                        new Slot0Configs()
                                .withKP(PIDs.elevatorKP.getAsDouble())
                                .withKI(PIDs.elevatorKI.getAsDouble())
                                .withKD(PIDs.elevatorKD.getAsDouble())
                                .withKS(PIDs.elevatorKS.getAsDouble())
                                .withKG(PIDs.elevatorKG.getAsDouble())
                                .withKV(PIDs.elevatorKV.getAsDouble())
                                .withGravityType(GravityTypeValue.Elevator_Static)
                ).withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(PIDs.elevatorAcceleration.getAsDouble())
                                .withMotionMagicCruiseVelocity(PIDs.elevatorVelocity.getAsDouble())
                );
        elevator.getConfigurator().apply(config);
    }
}
