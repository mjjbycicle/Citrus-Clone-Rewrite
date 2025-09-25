package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.List;

import static edu.wpi.first.units.Units.Rotations;

public class PivotIOSpark implements PivotIO {
    private final SparkMax leaderSpark, followerSpark;
    private final CANcoder absoluteEncoder;
    
    private final StatusSignal<Angle> encoderAbsolutePositionRotations;
    private final StatusSignal<AngularVelocity> encoderAbsoluteVelocityRPS;
    private final List<Double> appliedVoltage;
    private final List<Double> supplyCurrent;
    private final List<Double> tempCelcius;

    public PivotIOSpark() {
        followerSpark =
                new SparkMax(PivotConstants.IDs.RightPivotID, SparkLowLevel.MotorType.kBrushless);
        leaderSpark = new SparkMax(PivotConstants.IDs.LeftPivotID, SparkLowLevel.MotorType.kBrushless);
        leaderSpark.configure(
                new SparkMaxConfig().inverted(true),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        followerSpark.configure(
                new SparkMaxConfig().follow(leaderSpark, true),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        absoluteEncoder = new CANcoder(PivotConstants.IDs.PivotEncoderID);
        absoluteEncoder
                .getConfigurator()
                .apply(
                        new MagnetSensorConfigs()
                                .withMagnetOffset(Rotations.of(0.5))
                                .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1)));

        encoderAbsolutePositionRotations = absoluteEncoder.getAbsolutePosition();
        encoderAbsoluteVelocityRPS = absoluteEncoder.getVelocity();
        appliedVoltage = List.of(leaderSpark.getBusVoltage(), followerSpark.getBusVoltage());
        supplyCurrent = List.of(leaderSpark.getOutputCurrent(), followerSpark.getOutputCurrent());
        tempCelcius = List.of(leaderSpark.getMotorTemperature(), followerSpark.getMotorTemperature());

        BaseStatusSignal.setUpdateFrequencyForAll(
                500, encoderAbsolutePositionRotations, encoderAbsoluteVelocityRPS);
        absoluteEncoder.optimizeBusUtilization(0, 1.0);
    }
    
    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.absoluteEncoderPositionRotations = encoderAbsolutePositionRotations.getValueAsDouble();
        inputs.velocityRotationsPerSecond = encoderAbsoluteVelocityRPS.getValueAsDouble();
        
        inputs.absoluteEncoderConnected = 
                BaseStatusSignal.refreshAll(
                                encoderAbsolutePositionRotations, encoderAbsoluteVelocityRPS)
                        .isOK();

        inputs.appliedVolts = appliedVoltage.stream().mapToDouble(d -> d).toArray();
        inputs.supplyCurrentAmps = supplyCurrent.stream().mapToDouble(d -> d).toArray();
        inputs.tempCelcius = tempCelcius.stream().mapToDouble(d -> d).toArray();
    }
    
    @Override
    public void runVolts(double volts) {
        leaderSpark.setVoltage(volts);
    }
    
    @Override
    public void setBrakeMode(boolean enabled) {
        leaderSpark.configure(
                new SparkMaxConfig().idleMode(
                        enabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast
                ),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );
        followerSpark.configure(
                new SparkMaxConfig().idleMode(
                        enabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast
                ),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );
    }
    
    @Override
    public void stop() {
        leaderSpark.stopMotor();
    }
}
