package frc.robot.constants.reef;

public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    private final String jsonFolder;

    FieldType(String jsonFolder) {
        this.jsonFolder = jsonFolder;
    }

    public String getJsonFolder() {
        return jsonFolder;
    }
}
