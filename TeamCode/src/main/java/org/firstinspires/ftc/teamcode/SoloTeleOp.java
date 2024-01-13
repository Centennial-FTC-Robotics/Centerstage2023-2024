package org.firstinspires.ftc.teamcode;

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
public class SoloTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot();
        robot.initialize(this);

        GamepadEx drivePad = new GamepadEx(gamepad1);

        int newPower = 0;
        double lastFrame = 0;

        waitForStart();
        while(opModeIsActive()) {
            robot.outtake.update();
            drivePad.readButtons();

            robot.drivetrain.drive(
                    drivePad.getLeftY(),
                    drivePad.getLeftX(),
                    drivePad.getRightX()
            );


//            robot.intake.setHeight(toolPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

            if(drivePad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                robot.intake.setHeight(5);
            }
            if(drivePad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                robot.intake.incHeight(-1);
            }

//            robot.outtake.setArm(toolPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

//            if(toolPad.wasJustPressed(GamepadKeys.Button.A)) {
//                newPower = robot.intake.cycleNoodles();
//            }

            robot.intake.setNoodlePower((
                    drivePad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                    drivePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))*0.8);


            if(drivePad.isDown(GamepadKeys.Button.B)) {
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


            if(drivePad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.outtake.incrementSlidePos(1);
            }
            if(drivePad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.outtake.retractSlides();
            }

            robot.drivetrain.setRotationLock(
                    drivePad.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON));
            double currFrame = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (currFrame - lastFrame));
            lastFrame = currFrame;
            telemetry.update();
        }



    }
}
