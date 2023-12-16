package org.centennialrobotics.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.centennialrobotics.Subsystem;
import org.centennialrobotics.processors.ElementProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

public class Camera extends Subsystem {

    double minArea = 100;

    Scalar blueL = new Scalar(75, 50, 0);
    Scalar blueH = new Scalar(120, 255, 255);


    Scalar redL = new Scalar(75, 50, 0);
    Scalar redH = new Scalar(120, 255, 255);

    Scalar low;
    Scalar high;

    VisionPortal portal;
    ElementProcessor processor;

    LinearOpMode opmode;


    @Override
    public void init(LinearOpMode opmode) {

    }

    public void init(boolean weAreBlue, LinearOpMode opmode) {
        this.opmode = opmode;
        if(weAreBlue) {
            low = blueL;
            high = blueH;
        } else {
            low = redL;
            high = redH;
        }

        processor = new ElementProcessor(
                low,
                high,
                () -> minArea,
                () -> 213,
                () -> 426
        );

        portal = new VisionPortal.Builder()
                .setCamera(opmode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .build();
    }

    public ElementProcessor.PropPositions detectElement() {
        long start = System.currentTimeMillis();
        ElementProcessor.PropPositions pos = processor.getRecordedPropPosition();
        while(opmode.opModeIsActive() &&
                System.currentTimeMillis()-start < 5000 &&
                pos == ElementProcessor.PropPositions.UNFOUND
        ) {
            pos = processor.getRecordedPropPosition();
        }


        return processor.getRecordedPropPosition();
    }
}
