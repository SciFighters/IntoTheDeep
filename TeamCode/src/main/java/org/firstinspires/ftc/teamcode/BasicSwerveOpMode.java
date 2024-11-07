

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(group = "swerve")
public class BasicSwerveOpMode extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private SwerveDrive swerveDrive;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    @Override
    public void runOpMode() {

        swerveDrive = new SwerveDrive(hardwareMap,multipleTelemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.dpad_up){
                swerveDrive.kp += 0.001;
            }
            if(gamepad1.dpad_down){
                swerveDrive.kp -= 0.001;
            }
            swerveDrive.Drive(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x);
            if(gamepad1.right_trigger > 0.2 && gamepad1.left_trigger > 0.2){
                swerveDrive.boost = 0.6;
            }
            else if(gamepad1.right_trigger > 0.2 || gamepad1.left_trigger > 0.2){
                swerveDrive.boost = 0.45;
            }
            else {
                swerveDrive.boost = 0.3;
            }
            telemetry.addData("lx: ",gamepad1.left_stick_x);
            telemetry.addData("ly: ",gamepad1.left_stick_y);
            telemetry.addData("rx: ",gamepad1.right_stick_x);
            telemetry.addData("heading",swerveDrive.getHeading());
            telemetry.addData("bl heading",swerveDrive.bl.getCurrentHeading());
            telemetry.update();

        }
    }
}
