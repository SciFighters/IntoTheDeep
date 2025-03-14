package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Point;

public class MecanumDriveBHI260AP extends SubsystemBase {
    public DcMotorEx fl, fr, bl, br;
    //    public Servo moverGoGo;
    public Servo moverServo;
    public Servo wentWentServo;
    MecanumDriveKinematics kinematics;
    BHI260IMU imu;
    double lastHeading = 0;
    int vl_LastPos = 0;
    int vr_LastPos = 0;
    int b_LastPos = 0;
    final double meters_to_inches = 39.37008;
    private final int ticksPerMeter = 13362;//tod
    private final double ticksPerDegree = 41.028;
    MecanumDriveWheelSpeeds wheelSpeeds;
    private final BHI260IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )

    );
    Point currentPos = new Point(0, 0);


    //bl = intakeOdometer, br = nonParallel, fr = dischargeOdometer
    ElapsedTime time = new ElapsedTime();

    double correctedHeading;
    boolean isFieldOriented = true;

    MultipleTelemetry telemetry;
    Point startingPosition;
    double forwardTicksPerMeter = 1817, strafeTicksPerMeter = 2048;
    double tickPerMeter = 1783;
    LinearOpMode opMode;
    double startAngle = 0;
    public double extraX = 0, extraY = 0, extraR = 0;
    double correctionX = 0, correctionY = 0;

    public MecanumDriveBHI260AP(MultipleTelemetry telemetry, HardwareMap hm, LinearOpMode opMode) {
        this.telemetry = telemetry;
        moverServo = hm.get(Servo.class, "movergogo");
        wentWentServo = hm.get(Servo.class, "moverwentwent");
        fl = hm.get(DcMotorEx.class, "fl_motor");
        fr = hm.get(DcMotorEx.class, "fr_motor");
        br = hm.get(DcMotorEx.class, "br_motor");
        bl = hm.get(DcMotorEx.class, "bl_motor");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vl_LastPos = 0;
        vr_LastPos = 0;
        b_LastPos = 0;
        lastHeading = 0;
//        distanceSensor = hm.get(DistanceSensor.class, "distanceSensor");
        this.opMode = opMode;
        imu = hm.get(BHI260IMU.class, "imu");
        this.currentPos = new Point(0, 0);
//        parameters.accelerationIntegrationAlgorithm = new IMU_Integrator(imu, startingPosition, startAngle, telemetry, bl, fr, fl);
        imu.initialize(parameters);

        Translation2d flLocation = new Translation2d(100, 164);//164
        Translation2d frLocation = new Translation2d(100, -164);//-164
        Translation2d brLocation = new Translation2d(-100, -164);//-164
        Translation2d blLocation = new Translation2d(-100, 164);//164
        kinematics = new MecanumDriveKinematics(flLocation, frLocation, blLocation, brLocation);
//        Translation2d OflLocation = new Translation2d(100, 164);
//        Translation2d OfrLocation = new Translation2d(100, -164);
//        Translation2d ObrLocation = new Translation2d(-100, -164);
//        Translation2d OblLocation = new Translation2d(-100, 164);
//        odometryKinematics = new MecanumDriveKinematics(OflLocation,OfrLocation,ObrLocation,OblLocation);


//        odometry = new MecanumDriveOdometry(odometryKinematics, Rotation2d.fromDegrees(getHeading()), new Pose2d(startingPosition.x,startingPosition.y,Rotation2d.fromDegrees(0)));
        time.reset();
//        imu.startAccelerationIntegration(new Position(DistanceUnit.METER, this.startingPosition.x, this.startingPosition.y, 0, 0), new Velocity(), 2);

    }

    public MecanumDriveBHI260AP(MultipleTelemetry telemetry, HardwareMap hm, Point start, double startAngle, LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = telemetry;
        moverServo = hm.get(Servo.class, "movergogo");
        wentWentServo = hm.get(Servo.class, "moverwentwent");
        fl = hm.get(DcMotorEx.class, "fl_motor");
        fr = hm.get(DcMotorEx.class, "fr_motor");
        br = hm.get(DcMotorEx.class, "br_motor");
        bl = hm.get(DcMotorEx.class, "bl_motor");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vl_LastPos = 0;
        vr_LastPos = 0;
        b_LastPos = 0;
        lastHeading = startAngle;
        this.currentPos = start;
        this.startAngle = startAngle;

        RobotLog.d("MecanumDrive: Init (%f, %f)", this.currentPos.x, this.currentPos.y);

//        Translation2d OflLocation = new Translation2d(100, 164);
//        Translation2d OfrLocation = new Translation2d(100, -164);
//        Translation2d ObrLocation = new Translation2d(-100, -164);
//        Translation2d OblLocation = new Translation2d(-100, 164);
//        odometryKinematics = new MecanumDriveKinematics(OflLocation,OfrLocation,ObrLocation,OblLocation);
        Translation2d flLocation = new Translation2d(164, 100);//164
        Translation2d frLocation = new Translation2d(164, -100);//-164
        Translation2d brLocation = new Translation2d(-164, -100);//-164
        Translation2d blLocation = new Translation2d(-164, 100);//164
        kinematics = new MecanumDriveKinematics(flLocation, frLocation, blLocation, brLocation);


        imu = hm.get(BHI260IMU.class, "imu");
        imu.initialize(parameters);

//        initIMU(imu, opMode);


//        odometry = new MecanumDriveOdometry(odometryKinematics, Rotation2d.fromDegrees(getHeading()), new Pose2d(start.x, start.y, startAngle));
        time.reset();

    }

    @Override
    public void periodic() {
        FS delta = getDeltaOdometerDistance();

        RobotLog.d("MecanumDrive: Update: (%f, %f) delta (%f, %f) ang: %f", this.currentPos.x, this.currentPos.y, delta.s, delta.f, getHeading());

        double a = -getHeading() / 180.0 * Math.PI;

        this.currentPos.x -= delta.f * Math.cos(a) - delta.s * Math.sin(a);
        this.currentPos.y -= delta.s * Math.cos(a) + delta.f * Math.sin(a);

    }

//    private void initIMU(BHI260IMU imu, LinearOpMode robot) {
//        this.imu = imu;
//
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        RobotLog.d("imu params init start");
//        parameters.accelerationIntegrationAlgorithm = new IMU_Integrator(imu, startingPosition,
//                startAngle, telemetry, bl, fr, fl);
//        RobotLog.d("imu init");
//        imu.initialize(parameters);
//        RobotLog.d("imu init finished");
//
//
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        while (!imu.isGyroCalibrated() && !robot.isStopRequested() && timer.seconds() < 5) {
//            robot.sleep(50);
//        }
//        if (imu.isGyroCalibrated()) {
//            robot.telemetry.addData("Gyro", "Done Calibrating");
//            RobotLog.d("Gyro done init");
//
//        } else {
//            robot.telemetry.addData("Gyro", "Gyro/IMU Calibration Failed");
//            RobotLog.d("Gyro failed init" + " " + imu.isGyroCalibrated() + " " + imu.isAccelerometerCalibrated() + " " + imu.isMagnetometerCalibrated());
//        }
//
//        imu.startAccelerationIntegration(new Position(DistanceUnit.METER, this.startingPosition.x, this.startingPosition.y, 0, 0), new Velocity(), 2);
//
//        RobotLog.d("IMU status: %s", imu.getSystemStatus().toShortString());
//        RobotLog.d("IMU calibration status: %s", imu.getCalibrationStatus().toString());
//    }


    public void drive(double x, double y, double rotation, double boost) {
        x += extraX;
        y += extraY;
        rotation += extraR;

        x *= boost * 3;
        y *= boost * 3;
        rotation *= boost * 2;

        ChassisSpeeds speeds = calcDriveSpeeds(x, y);


// Now use this in our kinematics
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        double[] speedsArr = {
                wheelSpeeds.frontLeftMetersPerSecond - rotation,
                wheelSpeeds.frontRightMetersPerSecond - rotation,
                wheelSpeeds.rearLeftMetersPerSecond + rotation,
                wheelSpeeds.rearRightMetersPerSecond + rotation};
        speedsArr = modulateSpeeds(speedsArr);
        double frontLeft = speedsArr[0];
        double frontRight = speedsArr[1];
        double backLeft = speedsArr[2];
        double backRight = speedsArr[3];
        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        bl.setPower(backLeft);
        br.setPower(backRight);
//        telemetry.addData("x",x);
//        telemetry.addData("y",y);
//        telemetry.addData("fl", fl.getCurrentPosition());
//        telemetry.addData("fr", fr.getCurrentPosition());
//        telemetry.addData("bl", bl.getCurrentPosition());
//        telemetry.addData("br", br.getCurrentPosition());
//        telemetry.addData("posx", imu.getPosition().x);
//        telemetry.addData("posy", imu.getPosition().y);

        telemetry.update();
    }

    public ChassisSpeeds calcDriveSpeeds(double x, double y) {
        if (isFieldOriented) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(-x, y, 0, Rotation2d.fromDegrees(getAdjustedHeading()));
        } else {
            return new ChassisSpeeds(-x, y, 0);
        }
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return ((-orientation.getYaw(AngleUnit.DEGREES)) % 360 + startAngle) % 360;
    }

    public void resetHeading() {
        correctedHeading = getHeading();
    }

    public void setHeading(double heading) {
        correctedHeading = heading + 180;
    }

    public double getAdjustedHeading() {
        return getHeading() + correctedHeading;
    }

    public Point getPosition() {
        return currentPos;
    }

    public void setFieldOriented(boolean fieldOriented) {
        this.isFieldOriented = fieldOriented;
    }

    public double[] modulateSpeeds(double[] speeds) {
        double max = Math.max(Math.max(Math.abs(speeds[0]), Math.abs(speeds[1])), Math.max(Math.abs(speeds[2]), Math.abs(speeds[3])));
        if (Math.abs(max) > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= max;
            }
        }
        return speeds;
    }

    public void setMoverServo(double pos) {
        moverServo.setPosition(pos);
    }

    public void setWentWentServo(double pos) {
        wentWentServo.setPosition(pos);
    }

    public void resetPos(Point pos) {
        this.currentPos = pos;
    }

    public FS getDeltaOdometerDistance() {
        int vl_tick = bl.getCurrentPosition();
        int vr_tick = fr.getCurrentPosition();
        int b_tick = fl.getCurrentPosition();

        RobotLog.d("MecanumDrive: getDelta: (%d, %d) delta (%d, %d) ang: %f", vl_tick, vr_tick, vl_LastPos, vr_LastPos, getHeading());

        double heading = getHeading();
        double headingDiff = heading - lastHeading;
        if (Math.abs(headingDiff) > 180) {
            headingDiff -= Math.signum(headingDiff) * 360;
        }
        double vl_dist = vl_tick - vl_LastPos;
        double vr_dist = vr_tick - vr_LastPos;
        double b_dist = b_tick - b_LastPos;
        double f = (vl_dist + vr_dist) / 2 / ticksPerMeter;
        double s = (b_dist + headingDiff * ticksPerDegree) / ticksPerMeter;
        lastHeading = heading;
        vl_LastPos = vl_tick;
        vr_LastPos = vr_tick;
        b_LastPos = b_tick;
        return new FS(f, s);

    }

    private class FS {
        public double f = 0;
        public double s = 0;

        FS(double f, double s) {
            this.f = f;
            this.s = s;
        }
    }


}
