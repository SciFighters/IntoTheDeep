package org.firstinspires.ftc.teamcode.subsystems;

public enum XRotationStages {
    UPPER(0.0),
    MIDDLE(0.5),
    LOWER(1.0);
    public final double POSITION;
    private XRotationStages(double position) {
        this.POSITION = position;
    }
}
