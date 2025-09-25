package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.Constants;

import static frc.robot.subsystems.pivot.PivotConstants.*;
import static frc.robot.subsystems.pivot.PivotConstants.PIDs.*;

public class PivotIOSim implements PivotIO {
    private static final double autoStartAngle = Units.degreesToRadians(80.0);
    
    private final SingleJointedArmSim sim =
            new SingleJointedArmSim(
                    DCMotor.getNEO(2),
                    gearing,
                    1/3.0* mass *Math.pow(length, 2),
                    Units.inchesToMeters(25),
                    PivotStates.getSimPosition(PivotStates.IDLE.position),
                    PivotStates.getSimPosition(PivotStates.BACK.position),
                    true,
                    Units.degreesToRadians(0.0));

    private final PIDController controller;
    private double appliedVoltage = 0.0;
    private boolean wasNotAuto = true;

    public PivotIOSim() {
        controller = new PIDController(pivotKP.getAsDouble(), pivotKI.getAsDouble(), pivotKD.getAsDouble());
        sim.setState(0.0, 0.0);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        // Reset at start of auto
        if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
            sim.setState(autoStartAngle, 0.0);
            wasNotAuto = false;
        }
        wasNotAuto = !DriverStation.isAutonomousEnabled();

        sim.update(Constants.loopPeriodSecs);

        inputs.absoluteEncoderPositionRotations = PivotStates.fromSimPosition(sim.getAngleRads());
        inputs.velocityRotationsPerSecond = Units.radiansToDegrees(sim.getVelocityRadPerSec());
        inputs.appliedVolts = new double[] {appliedVoltage};
        inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
        inputs.tempCelcius = new double[] {0.0};

        // Reset input
        sim.setInputVoltage(0.0);
    }

    @Override
    public void runVolts(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVoltage);
    }

    @Override
    public void runSetpoint(double setpointRads, double feedforward) {
        runVolts(controller.calculate(sim.getAngleRads(), setpointRads) + feedforward);
    }

    @Override
    public void setPID(double p, double i, double d) {
        controller.setPID(p, i, d);
    }

    @Override
    public void stop() {
        appliedVoltage = 0.0;
        sim.setInputVoltage(appliedVoltage);
    }
}