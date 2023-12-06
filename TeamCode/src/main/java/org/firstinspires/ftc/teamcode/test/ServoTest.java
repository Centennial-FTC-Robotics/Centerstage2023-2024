package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0);

        waitForStart();
        while(opModeIsActive()) {

            telemetry.addData("amt", gamepad1.right_trigger);
            servo.setPosition(gamepad1.right_trigger);
            telemetry.update();

        }
    }
}
