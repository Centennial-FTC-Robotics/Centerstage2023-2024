package org.centennialrobotics.util;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.centennialrobotics.subsystems.Drivetrain;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class CRAprilTagWrapperProcessor implements VisionProcessor {

    public static Pose2d storedPos;

    public AprilTagProcessor internalProcessor;
    public Drivetrain drivetrain;

    public CRAprilTagWrapperProcessor(AprilTagProcessor atagProcessor, Drivetrain dt) {
        this.internalProcessor = atagProcessor;
        this.drivetrain = dt;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        internalProcessor.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        storedPos = drivetrain.drivebase.getPoseEstimate();

        return internalProcessor.processFrame(frame, captureTimeNanos);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        internalProcessor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
    }

    public ArrayList<AprilTagDetection> freshDetections() {
        return internalProcessor.getFreshDetections();
    }
}
