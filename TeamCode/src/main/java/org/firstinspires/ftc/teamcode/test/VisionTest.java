package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.centennialrobotics.processors.ColorMassDetectionProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Config
@TeleOp
public class VisionTest extends LinearOpMode {

    public static boolean weAreBlue = true;
    public static double minArea = 100;

    private Scalar blueL = new Scalar(75, 50, 0);
    private Scalar blueH = new Scalar(120, 255, 255);


    private Scalar redL = new Scalar(75, 50, 0);
    private Scalar redH = new Scalar(120, 255, 255);


    @Override
    public void runOpMode() throws InterruptedException {

        Scalar low;
        Scalar high;

        if(weAreBlue) {
            low = blueL;
            high = blueH;
        } else {
            low = redL;
            high = redH;
        }

        ColorMassDetectionProcessor processor = new ColorMassDetectionProcessor(
                low,
                high,
                () -> minArea,
                () -> 213,
                () -> 426
        );

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .build();

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Currently Recorded Position", processor.getRecordedPropPosition());
            telemetry.addData("Camera State", portal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + processor.getLargestContourX() + ", y: " + processor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", processor.getLargestContourArea());
            telemetry.update();
        }



    }
}
