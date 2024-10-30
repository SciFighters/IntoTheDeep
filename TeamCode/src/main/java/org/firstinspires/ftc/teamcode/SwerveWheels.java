package org.firstinspires.ftc.teamcode;



public enum SwerveWheels {
    //372 wide 344 other wide
    FL(Math.atan(372/344)),
    FR(Math.PI - Math.atan(372/344)),
    BR(Math.PI + Math.atan(372/344)),
    BL(- Math.atan(372/344));

    public final double ANGLE;
    private SwerveWheels(double angle){
        this.ANGLE = angle;
    }
}
