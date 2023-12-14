package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class BucketTest extends LinearOpMode {

//    public static double POS = 0.5;
    public static double LOW = 0.35;
    public static double HIGH = 0.65;



    public void runOpMode() throws InterruptedException {

        Servo bucketServo = hardwareMap.get(Servo.class, "outtakeServoL");
        CRServo wheel = hardwareMap.get(CRServo.class, "wheel");

        waitForStart();
        while(opModeIsActive()) {
            bucketServo.setPosition(gamepad1.right_trigger*(HIGH-LOW)+LOW);
            wheel.setPower(gamepad1.left_stick_y);
        }

    }
}
