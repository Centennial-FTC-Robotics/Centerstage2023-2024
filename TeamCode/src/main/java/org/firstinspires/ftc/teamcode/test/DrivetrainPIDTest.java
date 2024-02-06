package org.firstinspires.ftc.teamcode.test;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;


@Config
@Photon
@TeleOp
public class DrivetrainPIDTest extends LinearOpMode {

    public static double transP = 0.1;
    public static double transI = 0.05;
    public static double transD = 0;
    public static double trans_iMax = .3;
    public static double trans_dMax = 999999999;
    public static double trans_filterGain = 0;

    public static double headP = 2;
    public static double headI = 0.2;
    public static double headD = 1000;
    public static double head_iMax = .2;
    public static double head_dMax = 999999999;
    public static double head_filterGain = 0.7;

    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetH = 0;
//    public static Pose2d target = new Pose2d(0, 0, 0);



    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        PIDCoefficientsEx transCoeffs = new PIDCoefficientsEx(
                transP, transI, transD, trans_iMax, trans_dMax, trans_filterGain
        );

        PIDCoefficientsEx headCoeffs = new PIDCoefficientsEx(
                headP, headI, headD, head_iMax, head_dMax, head_filterGain
        );

        PIDEx xPID = new PIDEx(transCoeffs);
        PIDEx yPID = new PIDEx(transCoeffs);
        AngleController hPID = new AngleController(new PIDEx(headCoeffs));

        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            Pose2d currPose = drive.getPoseEstimate();
            Pose2d target = new Pose2d(targetX, targetY, targetH);

            double x = xPID.calculate(target.getX(), currPose.getX());
            double y = yPID.calculate(target.getY(), currPose.getY());
            double h = hPID.calculate(target.getHeading(), currPose.getHeading());

            double xR = x * Math.cos(-currPose.getHeading()) - y * Math.sin(-currPose.getHeading());
            double yR = x * Math.sin(-currPose.getHeading()) + y * Math.cos(-currPose.getHeading());

            Pose2d power = new Pose2d(xR, yR, h);
            drive.setWeightedDrivePower(power);

            transCoeffs.Kp = transP; transCoeffs.Ki = transI; transCoeffs.Kd = transD;
            transCoeffs.maximumIntegralSum = trans_iMax;
            transCoeffs.stabilityThreshold = trans_dMax; transCoeffs.lowPassGain = trans_filterGain;

            headCoeffs.Kp = headP; headCoeffs.Ki = headI; headCoeffs.Kd = headD;
            headCoeffs.maximumIntegralSum = head_iMax;
            headCoeffs.stabilityThreshold = head_dMax; headCoeffs.lowPassGain = head_filterGain;

            telemetry.update();

        }


    }
}
