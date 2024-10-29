

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp(group = "swerve")
public class BasicSwerveOpMode extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private SwerveDrive swerveDrive;

    @Override
    public void runOpMode() {

        swerveDrive = new SwerveDrive(hardwareMap,telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            swerveDrive.Drive(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x);
            if(gamepad1.right_trigger > 0.2 && gamepad1.left_trigger > 0.2){
                swerveDrive.speed = 0.6;
            }
            else if(gamepad1.right_trigger > 0.2 || gamepad1.left_trigger > 0.2){
                swerveDrive.speed = 0.45;
            }
            else {
                swerveDrive.speed = 0.3;
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
