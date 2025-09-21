package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.TunableProfiledPIDController;
import lib.TunableTrapezoidProfile;

public class PivotSubsystem extends SubsystemBase {
    private final SparkMax pivot;
    private final CANcoder pivotEncoder;
    private final TunableProfiledPIDController pidController;

    private PivotConstants.PivotStates pivotState = PivotConstants.PivotStates.IDLE;

    public PivotSubsystem() {
        SparkMax rightPivot =
                new SparkMax(PivotConstants.IDs.RightPivotID, SparkLowLevel.MotorType.kBrushless);
        pivot = new SparkMax(PivotConstants.IDs.LeftPivotID, SparkLowLevel.MotorType.kBrushless);
        pivot.configure(
                new SparkMaxConfig().inverted(true),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        rightPivot.configure(
                new SparkMaxConfig().follow(pivot, true),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        pivotEncoder = new CANcoder(PivotConstants.IDs.PivotEncoderID);
        pivotEncoder
                .getConfigurator()
                .apply(
                        new MagnetSensorConfigs()
                                .withMagnetOffset(Rotations.of(0.5))
                                .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1)));
        pidController =
                new TunableProfiledPIDController(
                        PivotConstants.PIDs.pivotKP,
                        PivotConstants.PIDs.pivotKI,
                        PivotConstants.PIDs.pivotKD,
                        new TunableTrapezoidProfile.TunableConstraints(
                                PivotConstants.PIDs.pivotVelocity, PivotConstants.PIDs.pivotAcceleration));
        pidController.setGoal(new TunableTrapezoidProfile.State(pivotState.position, 0));
    }

    public Command runCurrentState() {
        return this.run(
                () -> {
                    double pidOutput =
                            pidController.calculate(pivotEncoder.getAbsolutePosition().getValueAsDouble());
                    SmartDashboard.putNumber("Pivot PID Output", pidOutput);
                    pivot.set(pidOutput);
                });
    }

    public Command setState(PivotConstants.PivotStates newState) {
        return this.runOnce(
                () -> {
                    this.pivotState = newState;
                    pidController.setGoal(new TunableTrapezoidProfile.State(pivotState.position, 0));
                });
    }

    public Command setThenRunState(PivotConstants.PivotStates newState) {
        return setState(newState).andThen(runCurrentState());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Motor Output", pivot.getAppliedOutput());
        SmartDashboard.putNumber(
                "Pivot Position", pivotEncoder.getAbsolutePosition().getValueAsDouble());
    }
}
