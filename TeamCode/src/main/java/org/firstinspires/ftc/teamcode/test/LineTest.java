package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.centennialrobotics.Robot;
import org.centennialrobotics.subsystems.Drivetrain;
import org.centennialrobotics.subsystems.IMU;

@Config
@TeleOp
public class LineTest extends LinearOpMode {

    public static double headingP = 0;
    public static double driveP = 0;

    public static double targetHeading = 0;
    public static double targetPos = 0;

    public static double driveLimit = 0.6;
    public static double turnLimit = 0.2;

    public static boolean strafe = false;


    public void otherrunOpMode() throws InterruptedException{

        Robot robot = new Robot();
        robot.initialize(this);

        waitForStart();
        robot.drivetrain.driveDistance(targetPos);

    }

    public void runOpMode() throws InterruptedException {
        IMU imu = new IMU();
        Drivetrain dt = new Drivetrain();
        imu.init(this);
        dt.init(this);

        MultipleTelemetry tel = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()) {

            int[] encoderTicks = dt.getEncoderTicks();

            tel.addData("left", encoderTicks[0]);
            tel.addData("right", encoderTicks[1]);
            tel.addData("back", encoderTicks[2]);

            double currentPos = encoderTicks[0];
            if(strafe) {
                currentPos = dt.getEncoderTicks()[2];
            }

            double driveError = targetPos - currentPos;

            tel.addData("targetPos", targetPos);
            tel.addData("currentPos", currentPos);

            double drivePower = Range.clip(driveError*driveP, -driveLimit, driveLimit);

            double currentAngle = imu.revIMU.getHeading();
            double turnError = targetHeading - currentAngle;

            tel.addData("targetHeading", targetHeading);
            tel.addData("currentHeading", currentAngle);
            tel.update();

            double turnPower = Range.clip(turnError*headingP, -turnLimit, turnLimit);

            if(strafe) {
                dt.drive(0, drivePower, turnPower, false);
            } else {
                dt.drive(drivePower, 0, turnPower, false);
            }


        }

    }
}
