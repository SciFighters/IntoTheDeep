package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoProgrammer extends LinearOpMode {
    Servo servo;
    double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.dpad_up) {
                position += 0.01;
            } else if (gamepad1.dpad_down) {
                position -= 0.01;
            }
            servo.setPosition(position);
            telemetry.addData("pos",position);
            telemetry.update();
        }
    }
}
