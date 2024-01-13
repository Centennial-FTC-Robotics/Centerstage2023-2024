package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.centennialrobotics.Robot;
import org.centennialrobotics.subsystems.Intake;
import org.centennialrobotics.subsystems.Outtake;

@Photon
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
                    drivePad.getRightX()
            );

            if(drivePad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.drivetrain.mult *= -1;
            }

            if(drivePad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.drivetrain.mult *= 0.4;
            } else if(drivePad.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.drivetrain.mult = Math.signum(robot.drivetrain.mult);
            }


//            robot.intake.setHeight(toolPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

            if(toolPad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.intake.setHeight(5);
            }
            if(toolPad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.intake.incHeight(-1);
            }

//            robot.outtake.setArm(toolPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

//            if(toolPad.wasJustPressed(GamepadKeys.Button.A)) {
//                newPower = robot.intake.cycleNoodles();
//            }

            robot.intake.setNoodlePower(toolPad.getLeftY()*.65);


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

            if(gamepad2.y) {
                robot.climber.up();
            }

            if(gamepad2.x) {
                robot.climber.left();
            }

//            if(gamepad1.dpad_right) {
//                robot.climber.right();
//            }

            if(gamepad2.a) {
                robot.climber.down();
            }

            if(gamepad1.a) {
                robot.climber.setServoEnabled(true);
            }
            if(gamepad1.b) {
                robot.climber.setServoEnabled(false);
            }

//            if(gamepad1.right_bumper) {
//                robot.climber.launcher.setPower(0.2);
//            } else if(gamepad1.left_bumper) {
//                robot.climber.launcher.setPower(-0.2);
//            } else {
//                robot.climber.launcher.setPower(0);
//            }

            double hangMotorPower = (double)(gamepad2.right_trigger - gamepad2.left_trigger);
            if(hangMotorPower > 0) {
                robot.climber.setServoEnabled(false);
            }
            robot.climber.hangMotor.setPower(hangMotorPower);

        }

    }
}
