package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AprilTagLocTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal vision = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();

        vision.setProcessorEnabled(aprilTag, false);
        waitForStart();
        while(opModeIsActive()) {

            if(vision.getProcessorEnabled(aprilTag)) {
                ArrayList<AprilTagDetection> detections = aprilTag.getFreshDetections();

                if(detections != null) {
                    VectorF fieldPos = detections.get(0).metadata.fieldPosition;

                }

            }


        }
    }
}
