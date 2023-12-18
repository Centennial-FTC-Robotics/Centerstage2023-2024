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

//            robot.drivetrain.strafeDistance(22);
            robot.drivetrain.turnToHeading(90);
            robot.drivetrain.driveDistance(-22);
            robot.intake.expelOne();
        } else if(pos == ElementProcessor.PropPositions.RIGHT) {
            robot.drivetrain.driveDistance(27);
            robot.drivetrain.turnToHeading(90);
            robot.intake.expelOne();
//            robot.drivetrain.strafeDistance(-22);
        } else if(pos == ElementProcessor.PropPositions.MIDDLE){
            robot.drivetrain.driveDistance(35);
            robot.drivetrain.driveDistance(-10);
            robot.intake.expelOne();
        } else {
            telemetry.addData("pos", "unkown");
            telemetry.update();
            while(opModeIsActive());
        }


    }

}
