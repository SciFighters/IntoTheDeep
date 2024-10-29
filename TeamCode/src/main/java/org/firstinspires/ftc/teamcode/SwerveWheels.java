package org.firstinspires.ftc.teamcode;



public enum SwerveWheels {
    FL(Math.atan(372/344)),
    FR(180 - Math.atan(372/344)),
    BR(180 + Math.atan(372/344)),
    BL(360 - Math.toRadians(372/344));

    public final double ANGLE;
    private SwerveWheels(double angle){
        this.ANGLE = angle;
    }
}
