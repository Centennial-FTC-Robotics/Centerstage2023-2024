package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.centennialrobotics.Robot;
import org.centennialrobotics.subsystems.Outtake;

@Autonomous
public class AutoBlueBackstage extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.initialize(this);

        waitForStart();
        robot.drivetrain.driveDistance(-25);
        robot.drivetrain.turnToHeading(-90);
        robot.intake.expelOne();
        robot.drivetrain.strafeDistance(24);
        robot.drivetrain.turnToHeading(90);
        robot.drivetrain.driveDistance(-3*24);

        robot.drivetrain.strafeDistance(1.25*23.75);

        robot.outtake.incrementSlidePos(1);
        long start = System.currentTimeMillis();
        robot.drivetrain.drive(-0.15, 0, 0, false);
        while(opModeIsActive() && System.currentTimeMillis() - start < 5000) {
            robot.outtake.update();
        }
        robot.drivetrain.drive(0, 0, 0, false);
        robot.outtake.setWheel(Outtake.wheelOutDir);
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
