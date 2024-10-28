package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(group = "tests")
public class WheelTest extends LinearOpMode {
    // bl: servo = 1, analog = 0, motor = 0
    // br: servo = 4, analog = 2,  motor = 3
    // fl: servo = 0, analog = 1, motor = 1
    // fr: servo = 3, analog = 3, motor = 2
    SwerveModule bl,br, fl, fr,cr;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    @Override
    public void runOpMode(){
        bl = new SwerveModule(hardwareMap.get(DcMotor.class, "bl_motor"),
                hardwareMap.get(CRServo.class, "bl_servo"),
                hardwareMap.analogInput.get("bl_encoder"));
        br = new SwerveModule(hardwareMap.get(DcMotor.class, "br_motor"),
                hardwareMap.get(CRServo.class, "br_servo"),
                hardwareMap.analogInput.get("br_encoder"));
        fl = new SwerveModule(hardwareMap.get(DcMotor.class, "fl_motor"),
                hardwareMap.get(CRServo.class, "fl_servo"),
                hardwareMap.analogInput.get("fl_encoder"));
        fr = new SwerveModule(hardwareMap.get(DcMotor.class, "fr_motor"),
                hardwareMap.get(CRServo.class, "fr_servo"),
                hardwareMap.analogInput.get("fr_encoder"));
        cr = fl;
        // FLONG
        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.y){
                cr.setPower(0);
                cr = fl;
            } else if(gamepad1.x) {
                cr.setPower(0);
                cr = bl;
            } else if(gamepad1.b) {
                cr.setPower(0);
                cr = fr;
            } else if(gamepad1.a) {
                cr.setPower(0);
                cr = br;
            }
            double power = -gamepad1.left_stick_y;
            double wantedAngle = Math.toDegrees(Math.atan2( -gamepad1.right_stick_y,gamepad1.right_stick_x));

            power = Utils.signSquare(power);

            cr.setHeading(wantedAngle);
            cr.setPower(power);
            cr.update();

            multipleTelemetry.addData("wanted angle: ", wantedAngle);
            multipleTelemetry.addData("X", gamepad1.right_stick_x);
            multipleTelemetry.addData("y", -gamepad1.right_stick_y);
            multipleTelemetry.addData("current angle: ", cr.getCurrentHeading());
            multipleTelemetry.addData("motor position: ",cr.getPosition());
            multipleTelemetry.update();

        }
    }
}
