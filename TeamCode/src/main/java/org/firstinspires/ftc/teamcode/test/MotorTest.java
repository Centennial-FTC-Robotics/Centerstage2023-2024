package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "test");

        waitForStart();
        while(opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
        }

    }
}
