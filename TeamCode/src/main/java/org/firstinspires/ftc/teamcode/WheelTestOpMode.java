package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class SteeringServoDorel {
    private CRServo servo;
    private AnalogInput encoder;
    private double targetAngle = 0;
    private final double MAX_VOLTAGE = 3.27;
    private double e = 0; // current error
    private double w = -0.7; // weight
    private double p = 0.5; // proportional coefficient
    private double noiseCutoff = 1; // error in degrees where we stop adding noise
    private double noise = 10; // added noise in degrees to overcome deadzone
    public SteeringServoDorel(CRServo servo, AnalogInput encoder){
        this.servo = servo;
        this.encoder = encoder;
    }
    public void setTargetAngle(double angle){
        targetAngle = angle;
    }
    public double getTargetAngle(){
        return targetAngle;
    }
    public double getCurrentAngle(){
        return encoder.getVoltage() / MAX_VOLTAGE * 360;
    }
    public void update(){
        e = calcDeltaAngle(targetAngle, getCurrentAngle());
        if(Math.abs(e) > noiseCutoff){
            e += Math.random() * Math.signum(e) * noise;
        }
        double ne = -e/90;
        //double w = 1 - Math.abs(ne);
        double power = (w >= 0) ? (signroot(ne) * (w) + ne * (1 - w)) * p :
                (ne * Math.abs(ne) *(Math.abs(w)) + ne *(1 + w)) * p ;
        servo.setPower(power);
    }
    private double signroot(double x){
        return Math.sqrt(Math.abs(x)) * Math.signum(x);
    }
    private double calcDeltaAngle(double target, double current){
        double delta = target - current;
        if (Math.abs(delta) >= 180){
            delta -= Math.signum(delta) * 360;
        }
        return delta;
    }

}

@Config
@Disabled
public class WheelTestOpMode extends LinearOpMode {
    //best so far 0.35 for p and 0.75 for w
    //bottom left servo = 1
    //top left servo = 0
    //bottom right servo = 4
    //top right servo = 3
    //bottom left encoder = 0
    //top left encoder = 1
    //top right encoder = 2
    //bottom right encoder = 3
    //tl start at 156.2
    CRServo topLeftServo, topRightServo, bottomLeftServo, bottomRightServo;
    AnalogInput topLeftInput, topRightInput, bottomLeftInput, bottomRightInput;
    public static boolean iThing = false;
    public static double p = 0.25, w = -0.75;
    public static double noise = 10, noiseCutoff = 1;
    public static double iZone = 5, ki = 0, iResetPoint = 10;
    public static double period = 2;
    public static String wheel = "tl";
    public static boolean sin = false;
    double targetPos = 180;
    public static double target1 = 90,target2 = 180;
    double switchTime, timeOn90;
    private ElapsedTime runtime = new ElapsedTime();
    double i = 0;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private SteeringServoDorel flServo, frServo, blServo, brServo, currentServo;

    @Override
    public void runOpMode() throws InterruptedException {
        flServo = new SteeringServoDorel(hardwareMap.crservo.get("tl"),hardwareMap.analogInput.get("tla"));
        frServo = new SteeringServoDorel(hardwareMap.crservo.get("tr"),hardwareMap.analogInput.get("tra"));
        brServo = new SteeringServoDorel(hardwareMap.crservo.get("br"),hardwareMap.analogInput.get("bra"));
        blServo = new SteeringServoDorel(hardwareMap.crservo.get("bl"),hardwareMap.analogInput.get("bla"));


        waitForStart();

        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            switch(wheel){
                case "tl":
                    currentServo = flServo;
                    break;
                case "tr":
                    currentServo = frServo;
                    break;
                case "bl":
                    currentServo = blServo;
                    break;
                case "br":
                    currentServo = brServo;
                    break;

            }

            if (sin){
                targetPos = Math.sin(runtime.seconds() * Math.PI * period)
                        * (target2 - target1) / 2 +(target1 + target2)/2 ;
            }
            else if (runtime.seconds() >= switchTime + period) {
                if (targetPos == target2) {
                    targetPos = target1;
                } else {
                    targetPos = target2;
                }
                switchTime = runtime.seconds();
            }


//            if(Math.abs(targetPos - currentPos) * 0.1 < e){
//                timeOn90 = runtime.seconds() - switchTime;
//            }
//
            currentServo.setTargetAngle(targetPos);
            currentServo.update();
            dashboardTelemetry.addData("current", currentServo.getCurrentAngle());
            telemetry.addData("current", currentServo.getCurrentAngle());
            dashboardTelemetry.addData("target", targetPos);
            telemetry.addData("target", targetPos);
            dashboardTelemetry.addData("time on 90", timeOn90);
            dashboardTelemetry.update();
            telemetry.update();
        }
    }





}
