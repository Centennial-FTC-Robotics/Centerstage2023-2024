package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.centennialrobotics.subsystems.Drivetrain;
import org.centennialrobotics.subsystems.IMU;

@TeleOp
@Config
public class BlockerTest extends LinearOpMode {

    public static double leftPos = 0;
    public static double rightPos = 0;

    public void runOpMode() throws InterruptedException {

        Servo left = hardwareMap.get(Servo.class, "leftBlocker");
        Servo right = hardwareMap.get(Servo.class, "rightBlocker");
        IMU imu = new IMU();
        Drivetrain dt = new Drivetrain();
        dt.init(this);
        imu.init(this);

        waitForStart();

        while(opModeIsActive()) {
            dt.drivebase.getLocalizer().update();
            left.setPosition(leftPos);
            right.setPosition(rightPos);
            telemetry.addData("imu", -imu.revIMU.getHeading()*Math.PI/180.);
            telemetry.addData("odo", dt.drivebase.getLocalizer().getPoseEstimate().getHeading());
            telemetry.update();
        }

    }
}
