package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class HangTest extends LinearOpMode {

    public static double hangLow = 0;
    public static double hangHigh = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx hangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo hangArm = hardwareMap.get(Servo.class, "hangServo");


        waitForStart();
        while(opModeIsActive()) {

            hangMotor.setPower(gamepad1.left_stick_y);

            if(gamepad1.a) {
                hangArm.setPosition(hangHigh);
            } else {
                hangArm.setPosition(hangLow);
            }


        }

    }
}
