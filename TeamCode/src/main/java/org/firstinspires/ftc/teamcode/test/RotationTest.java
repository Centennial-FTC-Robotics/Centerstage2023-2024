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
public class RotationTest extends LinearOpMode {

    public static double P = 0.05;
    public static double limit = 0.5;
    public static double targetAngle = 0;



    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot();
        robot.initialize(this);

        waitForStart();
        robot.drivetrain.turnToHeading(targetAngle);

//        IMU imu = new IMU();
//        imu.init(this);
//
//        Drivetrain dt = new Drivetrain();
//        dt.init(this);
//
//        MultipleTelemetry tel = new MultipleTelemetry(
//                telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        waitForStart();
//        while(opModeIsActive()) {
//
//           double currentAngle = imu.revIMU.getHeading();
//           double error = currentAngle - targetAngle;
//
//            tel.addData("target", targetAngle);
//            tel.addData("pos", currentAngle);
//            tel.update();
//
//            double power = Range.clip(error*P, -limit, limit);
//
//            dt.drive(0, 0, power, false);
//
//        }

    }
}
