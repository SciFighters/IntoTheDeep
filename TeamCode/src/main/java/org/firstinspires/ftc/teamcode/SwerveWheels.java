package org.firstinspires.ftc.teamcode;



public enum SwerveWheels {
    //372 wide 344 other wide
    FL(Math.atan(372.0/344.0)),
    FR(Math.PI - Math.atan(372.0/344.0)),
    BR(Math.PI + Math.atan(372.0/344.0)),
    BL(- Math.atan(372.0/344.0));

    public final double ANGLE;
    private SwerveWheels(double angle){
        this.ANGLE = angle;
    }
}
