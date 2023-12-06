package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.centennialrobotics.subsystems.Drivetrain;

public class AprilTagLockDrive extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        Drivetrain drive = new Drivetrain();
        drive.init(this);

        waitForStart();

        while(opModeIsActive()) {

//            Gamepad

        }

    }
}
