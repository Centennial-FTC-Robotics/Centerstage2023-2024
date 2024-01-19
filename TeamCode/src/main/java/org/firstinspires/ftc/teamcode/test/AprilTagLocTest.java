package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.centennialrobotics.subsystems.Drivetrain;
import org.centennialrobotics.util.CRAprilTagWrapperProcessor;
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

@Photon
@TeleOp
public class AprilTagLocTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain dt = new Drivetrain();
        dt.init(this);

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        CRAprilTagWrapperProcessor processor = new CRAprilTagWrapperProcessor(aprilTag, dt);

        VisionPortal vision = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "tagCam"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(false)
                .build();

        waitForStart();
        while(opModeIsActive()) {



            if(vision.getProcessorEnabled(processor)) {
                ArrayList<AprilTagDetection> detections = processor.freshDetections();

                if(detections != null && detections.size() > 0) {
                    VectorF fieldPos = detections.get(0).metadata.fieldPosition;
                    AprilTagPoseFtc tagPose = detections.get(0).ftcPose;
                    telemetry.addData("x", tagPose.x);
                    telemetry.addData("y", tagPose.y);
                    telemetry.addData("yaw", tagPose.yaw);
                    telemetry.update();
                }

            }


        }
    }
}
