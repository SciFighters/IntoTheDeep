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
    SwerveModule swerveModule;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    @Override
    public void runOpMode(){
        swerveModule = new SwerveModule(hardwareMap.get(DcMotor.class, "motor"),
                hardwareMap.get(CRServo.class, "servo"),
                hardwareMap.analogInput.get("encoder"));

        waitForStart();

        while (opModeIsActive()){
            double power = -gamepad1.left_stick_y;
            double wantedAngle = gamepad1.right_stick_x * 90;

            power = Utils.signSquare(power);

            swerveModule.setHeading(wantedAngle);
            swerveModule.setPower(power);
            swerveModule.update();

             multipleTelemetry.addData("wanted angle: ", wantedAngle);
             multipleTelemetry.addData("current angle: ", swerveModule.getCurrentHeading());
             multipleTelemetry.addData("motor position: ",swerveModule.getPosition());
             multipleTelemetry.update();

        }
    }
}
