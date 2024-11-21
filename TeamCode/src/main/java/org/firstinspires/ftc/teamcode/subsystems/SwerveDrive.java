package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utils;

import lombok.Getter;

@Config
public class SwerveDrive extends SubsystemBase {
    private final ElapsedTime runtime = new ElapsedTime();

    SwerveModule bl, br, fl, fr;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    BNO055IMU imu;
    private double[] movementVector;
    private double[][] rotationVector;
    private double[][] wheelVectors;
    MultipleTelemetry telemetry;
    double boost = 0.35;
    final double rotationConpensation = 35.8;
    double wantedAngle = 0;
    boolean wasSpinning = false, isUsingAngleCorrection, wasTimeSelected;
    double noTurnElapsedTime = 0;
    double timeTillCorrection = 0.25;
    public static double kp = 0.02;
    public double[] position = {0,0};
    private final int ticksPerMeter = 1007;
    public static double minAngleError = 5, maxAngleError = 90;

    private double powerModifier;
    public SwerveDrive(HardwareMap hardwareMap, MultipleTelemetry telemetry) {

        this.telemetry = telemetry;

        bl = new SwerveModule(hardwareMap.get(DcMotor.class, "bl_motor"),
                hardwareMap.get(CRServo.class, "bl_servo"),
                hardwareMap.analogInput.get("bl_encoder"), 314.64);
        br = new SwerveModule(hardwareMap.get(DcMotor.class, "br_motor"),
                hardwareMap.get(CRServo.class, "br_servo"),
                hardwareMap.analogInput.get("br_encoder"), 54.93);
        fl = new SwerveModule(hardwareMap.get(DcMotor.class, "fl_motor"),
                hardwareMap.get(CRServo.class, "fl_servo"),
                hardwareMap.analogInput.get("fl_encoder"), 353.5);
        fr = new SwerveModule(hardwareMap.get(DcMotor.class, "fr_motor"),
                hardwareMap.get(CRServo.class, "fr_servo"),
                hardwareMap.analogInput.get("fr_encoder"), 346.9);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    //like update but here
    public void drive(double x, double y, double rotation, double _boost) {
        this.boost = _boost + 0.2;
        if (Math.hypot(x,y) < 0.05 && Math.abs(rotation) < 0.05){
            idle();
            return;
        }
        if (Math.abs(rotation) < 0.04) {
            if (wasSpinning) {
                noTurnElapsedTime = runtime.seconds();
            }
            wasSpinning = false;
            if (runtime.seconds() > noTurnElapsedTime + timeTillCorrection && !wasTimeSelected) {
                wasTimeSelected = true;
                isUsingAngleCorrection = true;
                wantedAngle = getHeading();
            }
            if (isUsingAngleCorrection) {
//                rotation += -Utils.signRoot(Utils.calcDeltaAngle(wantedAngle, getHeading()) * kp ) ;
                rotation += (Utils.calcDeltaAngle(wantedAngle, getHeading())) * kp;
            }

        } else {
            wasSpinning = true;
            wasTimeSelected = false;
            isUsingAngleCorrection = false;
        }

        movementVector = rotateVectors(boost * x, boost * y, getAdjustedHeading(rotation));  // Convert field speed to robot coordinates
        rotationVector = getRotationVectors(rotation);
        wheelVectors = addVectors(rotationVector, movementVector);
        for (int i = 0; i < 4; i++) {
            wheelVectors[i] = cart2polar(wheelVectors[i]);
        }
        double angleError = Math.max(Math.max(Math.abs(fl.getAngleError()),Math.abs(fr.getAngleError())),
                                Math.max(Math.abs(bl.getAngleError()), Math.abs(br.getAngleError())));
        powerModifier = Utils.map(angleError, minAngleError, maxAngleError, 1, 0);
        modulateSpeeds(wheelVectors);
        updateSwerveModules(wheelVectors);
        updatePos();
        telemetry.addData("power modifier", powerModifier);
        telemetry.addData("maxAngleError", angleError);
        telemetry.addData("posX", position[0]);
        telemetry.addData("posY", position[1]);
        telemetry.update();

    }

    private void idle() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.update();
        fr.update();
        br.update();
        bl.update();
    }

    private void telemetry() {
        telemetry.addData("fl power", fl.getPower());
        telemetry.addData("fr power", fr.getPower());
        telemetry.addData("bl power", bl.getPower());
        telemetry.addData("br power", br.getPower());
        telemetry.addData("fl wanted", fl.getTargetHeading());
        telemetry.addData("fr wanted", fr.getTargetHeading());
        telemetry.addData("bl wanted", br.getTargetHeading());
        telemetry.addData("fl current", fl.getCurrentHeading());
        telemetry.addData("fr current", fr.getCurrentHeading());
        telemetry.addData("bl current", bl.getCurrentHeading());
        telemetry.addData("br current", fr.getCurrentHeading());
        telemetry.addData("rotationConpensation: ", rotationConpensation);
        telemetry.addData("heading", getHeading());
        telemetry.update();
    }

    //rotate a vector by an angle for field oriented
    private double[] rotateVectors(double x, double y, double heading) {
        heading = Math.toRadians(heading);
        double[] vectors = {0, 0};
        vectors[0] = Math.cos(heading) * x - Math.sin(heading) * y;
        vectors[1] = Math.sin(heading) * x + Math.cos(heading) * y;
        return vectors;

    }

    //returns rotation vectors adjusted to each wheel in a multidimensional array
    //FL is 0, FR is 1, BR is 2, BL is 3.
    private double[][] getRotationVectors(double rotation) {
        rotation *= boost * rotation * Math.signum(rotation);
        double[][] rotationVector = {
                {Math.sin(SwerveWheels.FL.ANGLE) * rotation, Math.cos(SwerveWheels.FL.ANGLE) * rotation},
                {Math.sin(SwerveWheels.FR.ANGLE) * rotation, Math.cos(SwerveWheels.FR.ANGLE) * rotation},
                {Math.sin(SwerveWheels.BR.ANGLE) * rotation, Math.cos(SwerveWheels.BR.ANGLE) * rotation},
                {Math.sin(SwerveWheels.BL.ANGLE) * rotation, Math.cos(SwerveWheels.BL.ANGLE) * rotation}};
        return rotationVector;
    }

    //adds the rotationVector and the movement vector making it what each wheel needs to do
    private double[][] addVectors(double[][] rotationVector, double[] movementVector) {
        double[][] vector = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
        for (int i = 0; i < 4; i++) {
            vector[i][0] = rotationVector[i][0] + movementVector[0];
            vector[i][1] = rotationVector[i][1] + movementVector[1];
        }
        return vector;
    }

    //converts vector from x and y to length and angle(in degrees)
    private double[] cart2polar(double[] v) {
        double[] vector = {0, 0};
        //check if correct
        vector[0] = Math.hypot(v[0], v[1]); // Math.sqrt(x * x + y * y);
        vector[1] = Math.toDegrees(Math.atan2(v[0], v[1]));
        return vector;
    }

    //get chassis heading in degrees
    public double getHeading() {
        Orientation orientation = imu.getAngularOrientation();
        return (-orientation.firstAngle) % 360;
    }
    //for it to not go to the side when spinning and driving
    private double getAdjustedHeading(double rotation) {
        return getHeading() + rotationConpensation * rotation;
    }

    private void updateSwerveModules(double[][] wheelVectors) {

        fl.setHeading(wheelVectors[0][1]);
        fl.setPower(wheelVectors[0][0] * powerModifier);
        fr.setHeading(wheelVectors[1][1]);
        fr.setPower(wheelVectors[1][0] * powerModifier);
        br.setHeading(wheelVectors[2][1]);
        br.setPower(wheelVectors[2][0] * powerModifier);
        bl.setHeading(wheelVectors[3][1]);
        bl.setPower(wheelVectors[3][0] * powerModifier);
        fl.update();
        fr.update();
        br.update();
        bl.update();
    }

    private double[][] modulateSpeeds(double[][] vectors) {
        double a = Math.max(Math.max(Math.abs(vectors[0][0]), Math.abs(vectors[1][0])),
                Math.max(Math.abs(vectors[2][0]), Math.abs(vectors[3][0])));
        if (a > 1) {
            for (int i = 0; i < 4; i++) {
                vectors[i][0] /= a;
            }
        }
        return vectors;
    }
    private void updatePos(){
        double posDiff = fl.getPositionDifference();
        double angle = Math.toRadians(fl.getCurrentHeading());
        position[0] += Math.sin(angle) * posDiff /(4 * ticksPerMeter);
        position[1] += Math.cos(angle) * posDiff /(4 * ticksPerMeter);

    }


}
