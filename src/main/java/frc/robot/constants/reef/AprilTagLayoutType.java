package frc.robot.constants.reef;

import static frc.robot.constants.reef.Reef.fieldType;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.Constants;

import java.io.IOException;
import java.nio.file.Path;

public enum AprilTagLayoutType {
    OFFICIAL("2025-official"),
    NO_BARGE("2025-no-barge"),
    BLUE_REEF("2025-blue-reef"),
    RED_REEF("2025-red-reef"),
    NONE("2025-none");

    AprilTagLayoutType(String name) {
        if (Constants.disableHAL) {
            try {
                layout =
                        new AprilTagFieldLayout(
                                Path.of(
                                        "src",
                                        "main",
                                        "deploy",
                                        "apriltags",
                                        fieldType.getJsonFolder(),
                                        "2025-official.json"));
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        } else {
            try {
                layout =
                        new AprilTagFieldLayout(
                                Path.of(
                                        Filesystem.getDeployDirectory().getPath(),
                                        "apriltags",
                                        fieldType.getJsonFolder(),
                                        name + ".json"));
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }

        try {
            layoutString = new ObjectMapper().writeValueAsString(layout);
        } catch (JsonProcessingException e) {
            throw new RuntimeException(
                    "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
        }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;

    public AprilTagFieldLayout getLayout() {
        return layout;
    }
}
