package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.centennialrobotics.Robot;
import org.centennialrobotics.subsystems.Outtake;

@Config
@Photon
@TeleOp
public class MainTeleOp extends LinearOpMode {

    public static double intakeInMax = 0.65;
    public static double intakeOutMax = 0.5;
    public static double testIntakeHeight = 0;

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
                robot.drivetrain.mult = 1;
            }

            if(drivePad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.drivetrain.mult = -1;
            }




            if(drivePad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.drivetrain.mult *= 0.4;
            } else if(drivePad.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.drivetrain.mult = Math.signum(robot.drivetrain.mult);
            }


//            robot.intake.setHeight(toolPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

            if(toolPad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                robot.intake.incHeight(2);
//                robot.intake.setHeight(testIntakeHeight);
            }
            if(toolPad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                robot.intake.incHeight(-2);
            }

            robot.intake.setBumperUp(robot.intake.currentHeight == 0);

            if(robot.outtake.slidesTarget != robot.outtake.targets[0]) {
                robot.outtake.setManualSlidePower(toolPad.getLeftY()*.2);
            } else {
                robot.outtake.setManualSlidePower(0);
            }

//            robot.outtake.setArm(toolPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

//            if(toolPad.wasJustPressed(GamepadKeys.Button.A)) {
//                newPower = robot.intake.cycleNoodles();
//            }

            double intakePower = -toolPad.getRightY();

            if(Math.abs(intakePower) < 0.1) {
                intakePower = 0;
            }

            if(intakePower < 0) {
                intakePower *= intakeOutMax;
            } else {
                intakePower *= intakeInMax;
            }

//            if(robot.outtake.pos > 10)
//                intakePower = Range.clip(intakePower, -1, 0);

            robot.intake.setNoodlePower(intakePower);


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


            if(toolPad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.outtake.incrementSlidePos(1);
            }
            if(toolPad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.outtake.retractSlides();
            }
            if(gamepad2.left_stick_button && gamepad2.right_stick_button) {
                robot.climber.setHangLock(false);
            }
            if(gamepad2.y) {
                robot.climber.setHangLock(true);
            }



            if(drivePad.wasJustPressed(GamepadKeys.Button.A)) {
                robot.climber.setLauncherLift(!robot.climber.launcherLifted);
            }



            if(gamepad1.left_stick_button && gamepad1.right_stick_button) {
                robot.climber.launchPlane();
            }



//            if(gamepad1.right_bumper) {
//                robot.climber.launcher.setPower(0.2);
//            } else if(gamepad1.left_bumper) {
//                robot.climber.launcher.setPower(-0.2);
//            } else {
//                robot.climber.launcher.setPower(0);
//            }

            double hangMotorPower = (double)(gamepad2.right_trigger - gamepad2.left_trigger);
            robot.climber.hangMotor.setPower(hangMotorPower);

        }

    }
}
