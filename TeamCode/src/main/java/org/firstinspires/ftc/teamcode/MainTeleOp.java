package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.centennialrobotics.Robot;


@TeleOp
public class MainTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot();
        robot.initialize(this);

        GamepadEx drivePad = new GamepadEx(gamepad1);
        GamepadEx toolPad = new GamepadEx(gamepad2);

        waitForStart();
        while(opModeIsActive()) {
            robot.outtake.update();
            drivePad.readButtons();
            toolPad.readButtons();

            robot.drivetrain.drive(
                    -drivePad.getLeftY(),
                    drivePad.getLeftX(),
                    drivePad.getRightX(),
                    false
                    );

            robot.intake.setHeight(toolPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            robot.outtake.setArm(toolPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

            if(toolPad.wasJustPressed(GamepadKeys.Button.A)) {
                int newPower = robot.intake.cycleNoodles();
                robot.outtake.setWheel(Math.abs(newPower));
            }

            if(toolPad.isDown(GamepadKeys.Button.B)) {
                robot.outtake.setWheel(-1);
            }

        }

    }
}
