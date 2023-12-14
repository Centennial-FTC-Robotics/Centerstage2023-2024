package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.centennialrobotics.Robot;
import org.centennialrobotics.subsystems.Outtake;


@TeleOp
public class MainTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot();
        robot.initialize(this);

        GamepadEx drivePad = new GamepadEx(gamepad1);
        GamepadEx toolPad = new GamepadEx(gamepad2);

        int newPower = 0;

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
//            robot.outtake.setArm(toolPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

            if(toolPad.wasJustPressed(GamepadKeys.Button.A)) {
                newPower = robot.intake.cycleNoodles();
            }

            if(toolPad.isDown(GamepadKeys.Button.B)) {
                robot.outtake.setWheel(Outtake.wheelOutDir);
            } else if(robot.intake.noodleMotor.getPower() > 0.1){
                robot.outtake.setWheel(-Outtake.wheelOutDir);
            } else {
                robot.outtake.setWheel(0);
            }
//            } else if(newPower != 0) {
//                robot.outtake.setWheel(-Outtake.wheelOutDir);
//            } else {
//                robot.outtake.setWheel(0);
//            }


            if(toolPad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                robot.outtake.incrementSlidePos(1);
            }
            if(toolPad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                robot.outtake.retractSlides();
            }

            if(drivePad.wasJustPressed(GamepadKeys.Button.A)) {
                robot.climber.toggleHang();
            }

            int hangMotorPower = 0;
            if(drivePad.isDown(GamepadKeys.Button.DPAD_UP)) {
                hangMotorPower = 1;
            } else if(drivePad.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                hangMotorPower = -1;
            }
            robot.climber.hangMotor.setPower(hangMotorPower);

        }

    }
}
