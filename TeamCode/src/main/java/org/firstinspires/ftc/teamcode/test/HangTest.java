package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class HangTest extends LinearOpMode {

    public static double bottomServo = 0.16;
    public static double topServo = 1;

    public static boolean servosEnabled = true;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx hangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo armBottom = hardwareMap.get(Servo.class, "bottomHangServo");
        Servo armTop = hardwareMap.get(Servo.class, "topHangServo");

        CRServo launcher = hardwareMap.get(CRServo.class, "launcherServo");

        waitForStart();
        while(opModeIsActive()) {

            hangMotor.setPower(gamepad1.left_stick_y);
            launcher.setPower(gamepad1.right_stick_y);

            if(servosEnabled) {
                armBottom.getController().pwmEnable();
                armTop.getController().pwmEnable();

                armBottom.setPosition(bottomServo);
                armTop.setPosition(topServo);
            } else {
                armBottom.getController().pwmDisable();
                armTop.getController().pwmDisable();
            }



            if(gamepad1.dpad_up) {
                bottomServo = .5;
                topServo = 0;

                armBottom.setPosition(bottomServo);
                armTop.setPosition(topServo);
            }

            if(gamepad1.dpad_left) {
                bottomServo = .4;
                topServo = .2;

                armTop.setPosition(topServo);
                armBottom.setPosition(bottomServo);

            }

            if(gamepad1.dpad_right) {
                bottomServo = .5;
                topServo = .25;

                armTop.setPosition(topServo);
                armBottom.setPosition(bottomServo);

            }

            if(gamepad1.a) {
                servosEnabled = true;
            }
            if(gamepad1.b) {
                servosEnabled = false;
            }

            if(gamepad1.dpad_down) {
                bottomServo = .16;
                topServo = 1;

                armBottom.setPosition(bottomServo);
                armTop.setPosition(topServo);
            }


        }

    }
}
