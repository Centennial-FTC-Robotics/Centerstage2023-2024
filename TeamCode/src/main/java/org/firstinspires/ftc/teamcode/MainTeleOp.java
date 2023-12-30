package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.centennialrobotics.Robot;
import org.centennialrobotics.subsystems.Intake;
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
                    drivePad.getLeftY(),
                    drivePad.getLeftX(),
                    drivePad.getRightX(),
                    false
                    );


//            robot.intake.setHeight(toolPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

            if(gamepad2.y) {
                robot.intake.setHeight(Intake.liftHigh);
            } else if(gamepad2.x) {
                robot.intake.setHeight(Intake.liftMid);
            } else if(gamepad2.a) {
                robot.intake.setHeight(Intake.liftLow);
            }

//            robot.outtake.setArm(toolPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

//            if(toolPad.wasJustPressed(GamepadKeys.Button.A)) {
//                newPower = robot.intake.cycleNoodles();
//            }

            robot.intake.setNoodlePower(toolPad.getLeftY()*.7);

            robot.drivetrain.multiplier = 1- (.7*gamepad1.right_trigger);

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

            if(gamepad1.dpad_up) {
                robot.climber.up();
            }

            if(gamepad1.dpad_left) {
                robot.climber.left();
            }

            if(gamepad1.dpad_right) {
                robot.climber.right();
            }

            if(gamepad1.dpad_down) {
                robot.climber.down();
            }

            if(gamepad1.a) {
                robot.climber.setServoEnabled(true);
            }
            if(gamepad1.b) {
                robot.climber.setServoEnabled(false);
            }

            if(gamepad1.right_bumper) {
                robot.climber.launcher.setPower(0.2);
            } else if(gamepad1.left_bumper) {
                robot.climber.launcher.setPower(-0.2);
            } else {
                robot.climber.launcher.setPower(0);
            }

            int hangMotorPower = 0;
            if(drivePad.isDown(GamepadKeys.Button.Y)) {
                hangMotorPower = 1;
            } else if(drivePad.isDown(GamepadKeys.Button.X)) {
                hangMotorPower = -1;
            }
            robot.climber.hangMotor.setPower(hangMotorPower);

        }

    }
}
