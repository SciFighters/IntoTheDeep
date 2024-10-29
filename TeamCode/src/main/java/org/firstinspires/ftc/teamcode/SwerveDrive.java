package org.firstinspires.ftc.teamcode;

import android.webkit.WebView;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class SwerveDrive {
    SwerveModule bl,br,fl,fr;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    BNO055IMU imu;
    private double[] movementVector;
    private double[][] rotationVector;
    private double[][] wheelVectors;
    Telemetry telemetry;
    double speed = 0.35;
    public SwerveDrive(HardwareMap hardwareMap, Telemetry telemetry){
        bl = new SwerveModule(hardwareMap.get(DcMotor.class, "bl_motor"),
                hardwareMap.get(CRServo.class, "bl_servo"),
                hardwareMap.analogInput.get("bl_encoder"),147.85321100917432);
        br = new SwerveModule(hardwareMap.get(DcMotor.class, "br_motor"),
                hardwareMap.get(CRServo.class, "br_servo"),
                hardwareMap.analogInput.get("br_encoder"),77.94495412844036);
        fl = new SwerveModule(hardwareMap.get(DcMotor.class, "fl_motor"),
                hardwareMap.get(CRServo.class, "fl_servo"),
                hardwareMap.analogInput.get("fl_encoder"),44.697247706422026);
        fr = new SwerveModule(hardwareMap.get(DcMotor.class, "fr_motor"),
                hardwareMap.get(CRServo.class, "fr_servo"),
                hardwareMap.analogInput.get("fr_encoder"),84);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        this.telemetry = telemetry;


    }
    //like update but here
    public void Drive(double x, double y, double rotation){
        movementVector = rotateVectors(speed * x,speed * y,getHeading());
        rotationVector = getRotationVectors(rotation);
        wheelVectors = addVectors(rotationVector,movementVector);
        for (int i = 0; i < 4; i++){
            wheelVectors[i] = convertVector(wheelVectors[i][0], wheelVectors[i][1]);
        }
        modulateSpeeds();
        updateSwerveModules(wheelVectors);


    }
    //rotate a vector by an angle for field oriented
    private double[] rotateVectors(double x, double y, double angle){
        angle = Math.toRadians(angle);
        double[] vectors = {0,0};
        vectors[0] = Math.cos(angle) * x - Math.sin(angle) * y;
        vectors[1] = Math.sin(angle) * x + Math.cos(angle) * y;
        return vectors;

    }
    //returns rotation vectors adjusted to each wheel in a multidimensional array
    //FL is 0, FR is 1, BR is 2, BL is 3.
    private double[][] getRotationVectors(double rotation) {
        rotation *= speed;
        double[][] rotationVector = {
                {Math.sin(SwerveWheels.FL.ANGLE) * rotation,
                        Math.cos(SwerveWheels.FL.ANGLE) * rotation},
                {Math.sin(SwerveWheels.FR.ANGLE) * rotation,
                        Math.cos(SwerveWheels.FR.ANGLE) * rotation},
                {Math.sin(SwerveWheels.BR.ANGLE) * rotation,
                        Math.cos(SwerveWheels.BR.ANGLE) * rotation},
                {Math.sin(SwerveWheels.BL.ANGLE) * rotation,
                        Math.cos(SwerveWheels.BL.ANGLE) * rotation}};
        return rotationVector;
    }
    //adds the rotationVector and the movement vector making it what each wheel needs to do
    private double[][] addVectors(double[][] rotationVector, double[] movementVector){
        double[][] vector = {{0,0},{0,0},{0,0},{0,0}};
        for(int i = 0; i < 4; i++){
            vector[i][0] = rotationVector[i][0] + movementVector[0];
            vector[i][1] = rotationVector[i][1] + movementVector[1];
        }
        return vector;
    }
    //converts vector from x and y to length and angle(in degrees)
    private double[] convertVector(double x, double y){
        double[] vector = {0,0};
        //check if correct
        vector[1] = Math.toDegrees(Math.atan2(x,y));
        vector[0] = Math.sqrt(x * x + y * y);
        return vector;
    }
    //get chassis heading in degrees
    public double getHeading(){
        Orientation orientation = imu.getAngularOrientation();
        return (-orientation.firstAngle) % 360;
    }
    private void updateSwerveModules(double[][] wheelVectors){

        fl.setHeading(wheelVectors[0][1]);
        fl.setPower(wheelVectors[0][0]);
        fr.setHeading(wheelVectors[1][1]);
        fr.setPower(wheelVectors[1][0]);
        br.setHeading(wheelVectors[2][1]);
        br.setPower(wheelVectors[2][0]);
        bl.setHeading(wheelVectors[3][1]);
        bl.setPower(wheelVectors[3][0]);
        fl.update();
        fr.update();
        br.update();
        bl.update();
    }
    private void modulateSpeeds(){
        double a =Math.abs(Math.max(Math.max(wheelVectors[0][0],wheelVectors[1][0]),Math.max(wheelVectors[2][0],wheelVectors[3][0])));
        if (a > 1){
            for (int i = 0; i < 4; i++){
                wheelVectors[i][0] /= a;
            }

        }

    }


}
