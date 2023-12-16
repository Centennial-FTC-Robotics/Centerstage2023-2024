package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.centennialrobotics.Robot;
import org.centennialrobotics.processors.ElementProcessor;
import org.centennialrobotics.subsystems.Outtake;

@Autonomous
public class AutoRedBackstage extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.initialize(this);
        robot.camera.init(false, this);

        waitForStart();

        ElementProcessor.PropPositions pos = robot.camera.detectElement();
        if(pos == ElementProcessor.PropPositions.LEFT) {

            robot.drivetrain.driveDistance(27);
            robot.drivetrain.turnToHeading(-90);
            robot.intake.expelOne();
            robot.drivetrain.strafeDistance(7.5);
            robot.drivetrain.driveDistance(-26);
//            robot.drivetrain.strafeDistance(-22);
        } else if(pos == ElementProcessor.PropPositions.RIGHT) {
            robot.drivetrain.driveDistance(27);
            robot.drivetrain.turnToHeading(-90);
            robot.drivetrain.driveDistance(-22);
            robot.intake.expelOne();
            robot.drivetrain.strafeDistance(-5.5);
        } else if(pos == ElementProcessor.PropPositions.MIDDLE){
            robot.drivetrain.driveDistance(35);
            robot.drivetrain.driveDistance(-10);
            robot.intake.expelOne();
            robot.drivetrain.driveDistance(4);
            robot.drivetrain.turnToHeading(-90);
            robot.drivetrain.driveDistance(-26);

        } else {
            telemetry.addData("pos", "unkown");
            telemetry.update();
            while(opModeIsActive());
        }




//        robot.drivetrain.strafeDistance(1.25*23.75);

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
        while(opModeIsActive() && System.currentTimeMillis() - start < 7000) {
            robot.outtake.update();
        }



    }

}
