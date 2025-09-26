package frc.robot.constants.util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.pivot.PivotConstants;

public class JoystickAngleUtil {
    public static Rotation2d getAngle(double x, double y) {
        if (Math.abs(x) <= 0.01 && Math.abs(y) <= 0.01) {
            return Rotation2d.fromDegrees(0);
        }
        return new Rotation2d(-x, -y);
    }

    public static double getEncoderAngle(double x, double y) {
        Rotation2d actualAngle = getAngle(x, y);
        return PivotConstants.PivotStates.fromSimPosition(actualAngle.getRadians());
    }
}
