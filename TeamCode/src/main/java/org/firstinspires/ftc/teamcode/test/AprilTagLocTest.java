package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.centennialrobotics.subsystems.Drivetrain;
import org.centennialrobotics.util.CRAprilTagWrapperProcessor;
import org.centennialrobotics.util.CRPoseCalc;
import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
@Photon
@TeleOp
public class AprilTagLocTest extends LinearOpMode {

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

    public static Pose2d target = new Pose2d(56, -35.4, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain dt = new Drivetrain();
        dt.init(this);
        dt.drivebase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(903.79, 903.79, 699.758, 372.872)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SQPNP);

        CRAprilTagWrapperProcessor processor = new CRAprilTagWrapperProcessor(aprilTag, dt);

        VisionPortal vision = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "atagCam"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(processor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(false)
                .build();

        PIDCoefficientsEx transCoeffs = new PIDCoefficientsEx(
                transP, transI, transD, trans_iMax, trans_dMax, trans_filterGain
        );

        PIDCoefficientsEx headCoeffs = new PIDCoefficientsEx(
                headP, headI, headD, head_iMax, head_dMax, head_filterGain
        );

        PIDEx xPID = new PIDEx(transCoeffs);
        PIDEx yPID = new PIDEx(transCoeffs);
        AngleController hPID = new AngleController(new PIDEx(headCoeffs));

        dt.drivebase.setPoseEstimate(target);

        waitForStart();
        while(opModeIsActive()) {
            dt.drivebase.update();
            if(vision.getProcessorEnabled(processor)) {
                ArrayList<AprilTagDetection> detections = processor.freshDetections();

                Pose2d closest = null;
                double closestDist = Double.MAX_VALUE;

                if(detections != null && detections.size() > 0) {
                    for(AprilTagDetection detection : detections) {
                        if(detection.metadata == null) {
                            continue;
                        }
                        VectorF fieldPos = detection.metadata.fieldPosition;
                        AprilTagPoseFtc tagPose = detection.ftcPose;


                        Pose2d currPose = new Pose2d(
                                fieldPos.get(0)-tagPose.y,
                                fieldPos.get(1)+tagPose.x,
                                Math.toRadians(180-tagPose.yaw)
                        );

                        if(closest == null || CRPoseCalc.distance(currPose, closest) < closestDist) {
                            closest = currPose;
                            closestDist = CRPoseCalc.distance(currPose, closest);
                        }
                    }

                    if(closest != null)
                        dt.drivebase.setPoseEstimate(closest);
//
//                    telemetry.addData("x", tagPose.x);
//                    telemetry.addData("y", tagPose.y);
//                    telemetry.addData("yaw", tagPose.yaw);
//                    telemetry.update();
                }

            }

            Pose2d currPose = dt.drivebase.getPoseEstimate();

//            telemetry.addData("CurrPose", currPose.toString());
//            telemetry.addData("TargetPose", target.toString());
//            telemetry.update();


            double x = xPID.calculate(target.getX(), currPose.getX());
            double y = yPID.calculate(target.getY(), currPose.getY());
            double h = hPID.calculate(target.getHeading(), currPose.getHeading());

            double xR = x * Math.cos(-currPose.getHeading()) - y * Math.sin(-currPose.getHeading());
            double yR = x * Math.sin(-currPose.getHeading()) + y * Math.cos(-currPose.getHeading());

            Pose2d power = new Pose2d(xR, yR, h);

            if(gamepad1.right_bumper)
                dt.drivebase.setWeightedDrivePower(power);
            else dt.drivebase.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y
                    , -gamepad1.left_stick_x, -gamepad1.right_stick_x));


        }
    }
}
