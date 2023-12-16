package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.centennialrobotics.Robot;
import org.centennialrobotics.processors.ElementProcessor;
import org.centennialrobotics.subsystems.Outtake;

@Autonomous
public class AutoBlueFrontstage extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.initialize(this);
        robot.camera.init(true, this);

        waitForStart();

        ElementProcessor.PropPositions pos = robot.camera.detectElement();
        if(pos == ElementProcessor.PropPositions.LEFT) {
            robot.drivetrain.driveDistance(27);
            robot.drivetrain.turnToHeading(-90);
            robot.intake.expelOne();
            robot.drivetrain.strafeDistance(22);
            robot.drivetrain.turnToHeading(90);
        } else if(pos == ElementProcessor.PropPositions.RIGHT) {
            robot.drivetrain.driveDistance(27);
            robot.drivetrain.turnToHeading(90);
            robot.intake.expelOne();
            robot.drivetrain.strafeDistance(-22);
        } else if(pos == ElementProcessor.PropPositions.MIDDLE){
            telemetry.addData("pos", "mid");
            telemetry.update();
            while(opModeIsActive());
        } else {
            telemetry.addData("pos", "unkown");
            telemetry.update();
            while(opModeIsActive());
        }


        robot.drivetrain.driveDistance(-3*24);

        robot.drivetrain.strafeDistance(1.25*23.75);

        robot.outtake.incrementSlidePos(1);
        long start = System.currentTimeMillis();
        robot.drivetrain.drive(-0.15, 0, 0, false);
        while(opModeIsActive() && System.currentTimeMillis() - start < 5000) {
            robot.outtake.update();
        }
        robot.drivetrain.drive(0, 0, 0, false);
        robot.outtake.setWheel(Outtake.wheelOutDir*.2);
        Thread.sleep(2000);
        robot.outtake.setWheel(0);
        robot.drivetrain.driveDistance(5);

        robot.outtake.retractSlides();
        start = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - start < 5000) {
            robot.outtake.update();
        }


    }

}
