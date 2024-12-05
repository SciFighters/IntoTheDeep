package org.firstinspires.ftc.teamcode.subsystems;

public enum GripStages {
    TOP(0.0),
    MIDDLE(0.5),
    BOTTOM(1.0);
    public final double POSITION;
    private GripStages(double position){
        this.POSITION = position;
    }
}
