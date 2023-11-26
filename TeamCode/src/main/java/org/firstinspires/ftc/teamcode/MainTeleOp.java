package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.centennialrobotics.Robot;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.initialize(this);

        waitForStart();

        while(opModeIsActive()){
            robot.teleOpUpdate(gamepad1, gamepad2);
            telemetry.update();

        }
    }
}
