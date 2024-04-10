package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class BlockerTest extends LinearOpMode {

    public static double leftPos = 0;
    public static double rightPos = 0;

    public void runOpMode() throws InterruptedException {

        Servo left = hardwareMap.get(Servo.class, "leftBlocker");
        Servo right = hardwareMap.get(Servo.class, "rightBlocker");

        waitForStart();

        while(opModeIsActive()) {
            left.setPosition(leftPos);
            right.setPosition(rightPos);
        }

    }
}
