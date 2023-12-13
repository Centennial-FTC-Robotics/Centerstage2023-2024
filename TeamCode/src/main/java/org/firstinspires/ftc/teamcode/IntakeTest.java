package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx noodleMotor = hardwareMap.get(DcMotorEx.class, "noodleMotor");
        noodleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        noodleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo lift = hardwareMap.get(Servo.class, "leftIntake");
        lift.setDirection(Servo.Direction.REVERSE);

        CRServo wheel = hardwareMap.get(CRServo.class, "wheel");

        GamepadEx gamepad = new GamepadEx(gamepad1);

        boolean aToggle = false;

        waitForStart();
        double held = 0;
        while(opModeIsActive()) {
            gamepad.readButtons();
            double motorPower = gamepad1.left_stick_y;

            if(gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                aToggle = !aToggle;
            }

            if(aToggle) {
                if(gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    held = motorPower;
                }
                motorPower = held;
            }

            noodleMotor.setPower(motorPower);
            wheel.setPower(motorPower);
            lift.setPosition(gamepad1.right_trigger);



        }

    }
}
