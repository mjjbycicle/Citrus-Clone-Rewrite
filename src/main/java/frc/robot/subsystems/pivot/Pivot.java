package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.pivot.PivotConstants.*;
import static frc.robot.subsystems.pivot.PivotConstants.PIDs.*;

public class Pivot {

    // Profile constraints
    public static final Supplier<TrapezoidProfile.Constraints> maxProfileConstraints =
            () -> new TrapezoidProfile.Constraints(pivotVelocity.getAsDouble(), pivotAcceleration.getAsDouble());

    @AutoLogOutput private PivotStates goal = PivotStates.IDLE;
    private boolean characterizing = false;

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private TrapezoidProfile.Constraints currentConstraints = maxProfileConstraints.get();
    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    private double goalAngle;
    private ArmFeedforward ff;

    private final PivotVisualizer measuredVisualizer;
    private final PivotVisualizer setpointVisualizer;
    private final PivotVisualizer goalVisualizer;

    private BooleanSupplier disableSupplier = DriverStation::isDisabled;
    private BooleanSupplier coastSupplier = () -> false;
    private BooleanSupplier halfStowSupplier = () -> true;
    private boolean brakeModeEnabled = true;

    private boolean wasNotAuto = false;

    public Pivot(PivotIO io) {
        this.io = io;
        io.setBrakeMode(true);

        profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(pivotVelocity.getAsDouble(), pivotAcceleration.getAsDouble()));
        io.setPID(kP.get(), kI.get(), kD.get());
        ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        // Set up visualizers
        NoteVisualizer.setArmAngleSupplier(() -> Rotation2d.fromRadians(inputs.positionRads));
        measuredVisualizer = new ArmVisualizer("Measured", Color.kBlack);
        setpointVisualizer = new ArmVisualizer("Setpoint", Color.kGreen);
        goalVisualizer = new ArmVisualizer("Goal", Color.kBlue);
    }

    public void setOverrides(
            BooleanSupplier disableOverride,
            BooleanSupplier coastOverride,
            BooleanSupplier halfStowOverride) {
        disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
        coastSupplier = coastOverride;
        halfStowSupplier = halfStowOverride;
    }

    private double getStowAngle() {
        if (DriverStation.isTeleopEnabled()
                && RobotState.getInstance().inCloseShootingZone()
                && !halfStowSupplier.getAsBoolean()) {
            return MathUtil.clamp(
                    setpointState.position,
                    minAngle.getRadians(),
                    Units.degreesToRadians(partialStowUpperLimitDegrees.get()));
        } else {
            return minAngle.getRadians();
        }
    }

    public void periodic() {
        // Process inputs
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        // Set alerts
        leaderMotorDisconnected.set(!inputs.leaderMotorConnected);
        followerMotorDisconnected.set(!inputs.followerMotorConnected);
        absoluteEncoderDisconnected.set(!inputs.absoluteEncoderConnected);

        // Update controllers
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
                kS,
                kG,
                kV,
                kA);

        // Check if disabled
        // Also run first cycle of auto to reset arm
        if (disableSupplier.getAsBoolean()
                || (Constants.getMode() == Constants.Mode.SIM
                && DriverStation.isAutonomousEnabled()
                && wasNotAuto)) {
            io.stop();
            // Reset profile when disabled
            setpointState = new TrapezoidProfile.State(inputs.positionRads, 0);
        }
        Leds.getInstance().armEstopped = disableSupplier.getAsBoolean() && DriverStation.isEnabled();
        // Track autonomous enabled
        wasNotAuto = !DriverStation.isAutonomousEnabled();

        // Set coast mode with override
        setBrakeMode(!coastSupplier.getAsBoolean());
        Leds.getInstance().armCoast = coastSupplier.getAsBoolean();

        // Don't run profile when characterizing, coast mode, or disabled
        if (!characterizing && brakeModeEnabled && !disableSupplier.getAsBoolean()) {
            // Run closed loop
            goalAngle =
                    goal.getRads() + (goal == Goal.AIM ? Units.degreesToRadians(currentCompensation) : 0.0);
            if (goal == Goal.STOW) {
                goalAngle = getStowAngle();
            }
            setpointState =
                    profile.calculate(
                            Constants.loopPeriodSecs,
                            setpointState,
                            new TrapezoidProfile.State(
                                    MathUtil.clamp(
                                            goalAngle,
                                            Units.degreesToRadians(lowerLimitDegrees.get()),
                                            Units.degreesToRadians(upperLimitDegrees.get())),
                                    0.0));
            if (goal == Goal.STOW
                    && EqualsUtil.epsilonEquals(goalAngle, minAngle.getRadians())
                    && atGoal()) {
                io.stop();
            } else {
                io.runSetpoint(
                        setpointState.position, ff.calculate(setpointState.position, setpointState.velocity));
            }

            goalVisualizer.update(goalAngle);
            Logger.recordOutput("Arm/GoalAngle", goalAngle);
        }

        // Logs
        measuredVisualizer.update(inputs.positionRads);
        setpointVisualizer.update(setpointState.position);
        Logger.recordOutput("Arm/SetpointAngle", setpointState.position);
        Logger.recordOutput("Arm/SetpointVelocity", setpointState.velocity);
        Logger.recordOutput("Superstructure/Arm/Goal", goal);
    }

    public void stop() {
        io.stop();
    }

    @AutoLogOutput(key = "Superstructure/Arm/AtGoal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(setpointState.position, goalAngle, 1e-3);
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public void setProfileConstraints(TrapezoidProfile.Constraints constraints) {
        if (EqualsUtil.epsilonEquals(currentConstraints.maxVelocity, constraints.maxVelocity)
                && EqualsUtil.epsilonEquals(currentConstraints.maxAcceleration, constraints.maxVelocity))
            return;
        currentConstraints = constraints;
        profile = new TrapezoidProfile(currentConstraints);
    }

    public void runCharacterization(double amps) {
        characterizing = true;
        io.runCurrent(amps);
    }

    public double getCharacterizationVelocity() {
        return inputs.velocityRadsPerSec;
    }

    public void endCharacterization() {
        characterizing = false;
    }
}