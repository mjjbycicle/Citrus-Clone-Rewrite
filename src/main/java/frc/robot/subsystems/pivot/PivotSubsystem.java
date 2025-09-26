package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.TunableArmFeedforward;
import lib.TunableProfiledPIDController;
import lib.TunableTrapezoidProfile;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.pivot.PivotConstants.IDs;
import static frc.robot.subsystems.pivot.PivotConstants.PIDs;

public class PivotSubsystem extends SubsystemBase {
    private final SparkMax pivot;
    private final CANcoder pivotEncoder;
    private final TunableProfiledPIDController pidController;
    private final TunableArmFeedforward armFeedforward;

    private PivotConstants.PivotStates pivotState = PivotConstants.PivotStates.IDLE;

    public PivotSubsystem() {
        SparkMax rightPivot =
                new SparkMax(IDs.RightPivotID, SparkLowLevel.MotorType.kBrushless);
        pivot = new SparkMax(IDs.LeftPivotID, SparkLowLevel.MotorType.kBrushless);
        pivot.configure(
                new SparkMaxConfig().inverted(false),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        rightPivot.configure(
                new SparkMaxConfig().follow(pivot, true),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        pivotEncoder = new CANcoder(IDs.PivotEncoderID);
        pivotEncoder
                .getConfigurator()
                .apply(
                        new MagnetSensorConfigs()
                                .withMagnetOffset(Rotations.of(0.5))
                                .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1)));
        pidController =
                new TunableProfiledPIDController(
                        PIDs.pivotKP,
                        PIDs.pivotKI,
                        PIDs.pivotKD,
                        new TunableTrapezoidProfile.TunableConstraints(
                                PIDs.pivotVelocity, PIDs.pivotAcceleration));
        pidController.setGoal(new TunableTrapezoidProfile.State(pivotState.position, 0));
        pidController.reset(getEncoderPosition(), 0);
        armFeedforward = new TunableArmFeedforward(PIDs.pivotKS, PIDs.pivotKG, PIDs.pivotKV);
    }

    public Command runCurrentState() {
        return this.run(
                        () -> {
                            if (pivotState == PivotConstants.PivotStates.JOYSTICK_LISTEN) {
                                pidController.setGoal(getGoal());
                            }
                            double pidOutput =
                                    pidController.calculate(getEncoderPosition())
                                    + armFeedforward.calculate(
                                            getActualPositionRadians(),
                                            pidController.getSetpoint().velocity
                                    );
                            SmartDashboard.putNumber("Pivot PID Output", pidOutput);
                            pivot.set(pidOutput);
                        })
                .finallyDo(
                        () -> {
                            pivot.set(0);
                            pidController.setGoal(
                                    new TunableTrapezoidProfile.State(
                                            getEncoderPosition(), 0
                                    )
                            );
                            pidController.reset(getEncoderPosition(), 0);
                        });
    }

    public Command setState(PivotConstants.PivotStates newState) {
        return this.runOnce(
                () -> {
                    this.pivotState = newState;
                    pidController.setGoal(new TunableTrapezoidProfile.State(getGoal(), 0));
                });
    }

    public Command setThenRunState(PivotConstants.PivotStates newState) {
        return setState(newState).andThen(runCurrentState());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Motor Output", pivot.getAppliedOutput());
        SmartDashboard.putNumber("Pivot Position", getEncoderPosition());
        SmartDashboard.putNumber("Pivot Actual Angle", Degrees.convertFrom(
                getActualPositionRadians(),
                Radians
        ));
        SmartDashboard.putNumber("Pivot Feedforward Output", armFeedforward.calculate(
                getActualPositionRadians(),
                pidController.getSetpoint().velocity
        ));
        SmartDashboard.putNumber("Pivot Joystick Angle", PivotConstants.PivotStates.JOYSTICK_LISTEN.customPositionSupplier.getAsDouble());
    }

    public double getEncoderPosition() {
        return pivotEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getActualPositionRadians() {
        return PivotConstants.PivotStates.getSimPosition(getEncoderPosition());
    }

    public double getEncoderVelocity() {
        return pivotEncoder.getVelocity().getValueAsDouble();
    }

    public double getGoal() {
        if (pivotState == PivotConstants.PivotStates.JOYSTICK_LISTEN) return pivotState.customPositionSupplier.getAsDouble();
        return pivotState.position;
    }
}
