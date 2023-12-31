package com.example.meepmeepvisualization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.util.Objects;

import javax.imageio.ImageIO;

public class MeepMeepVisualization {

    public static void traj2() {
        MeepMeep meepMeep = new MeepMeep(900);

        double t = 23.5;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, 3.8, 3.8, 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-1.5*t, 2.5*t+4, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(-47.45, 37.48), Math.toRadians(270.00))
                                .addTemporalMarker(() -> {
//                                    intake.setHeight(5);
                                })
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-35.2, 34.76), Math.toRadians(270.00))

                                .splineTo(new Vector2d(-48.34, 12.39), Math.toRadians(191.31))
                                .addTemporalMarker(() -> {
//                                    intake.setNoodlePower(0.8);
//                                    outtake.setWheel(-1*Outtake.wheelOutDir);
                                })
                                .splineTo(new Vector2d(-60.5, 11.00), Math.toRadians(180.00))
                                .setReversed(false)
                                .setReversed(true)
                                .addTemporalMarker(() -> {
//                                    intake.setHeight(4);
                                })
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
//                                    intake.setNoodlePower(0);
//                                    outtake.setWheel(0);
                                })
                                .splineTo(new Vector2d(24.77, 11), Math.toRadians(1.49))
                                .addTemporalMarker(() -> {
//                                    outtake.incrementSlidePos(2);
                                })
                                .splineToConstantHeading(new Vector2d(52, 41), Math.toRadians(0.00))
                                .addTemporalMarker(() -> {
//                                    outtake.setWheel(Outtake.wheelOutDir);
                                })
                                .waitSeconds(0.5)
                                .setReversed(false)
                                .splineTo(new Vector2d(41.02, 28.63), Math.toRadians(252.35))
                                .addTemporalMarker(() -> {
//                                    outtake.retractSlides();
//                                    outtake.setWheel(0);
//                                    intake.setHeight(3);
//                                    intake.setNoodlePower(0.8);
//                                    outtake.setWheel(-1*Outtake.wheelOutDir);

                                })
                                .splineTo(new Vector2d(13.42, 11), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-60.5, 11.0), Math.toRadians(180.00))
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                                    intake.setHeight(2);
                                })
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
//                                    intake.setNoodlePower(0);
//                                    outtake.setWheel(0);
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(24.77, 11), Math.toRadians(1.49))
                                .addTemporalMarker(() -> {
//                                    outtake.incrementSlidePos(2);
                                })
                                .splineToConstantHeading(new Vector2d(52, 30), Math.toRadians(0.00))
                                .addTemporalMarker(() -> {
//                                    outtake.setWheel(Outtake.wheelOutDir);
                                })
                                .waitSeconds(0.5)
                                .setReversed(false)
                                .splineTo(new Vector2d(24.77, 11), Math.toRadians(181.5))
                                .addTemporalMarker(() -> {
//                                    outtake.retractSlides();
//                                    outtake.setWheel(0);
//                                    intake.setHeight(1);
//                                    intake.setNoodlePower(0.8);
//                                    outtake.setWheel(-1*Outtake.wheelOutDir);

                                })
                                .splineTo(new Vector2d(13.42, 11), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-60.5, 11.0), Math.toRadians(180.00))
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                                    intake.setHeight(0);
                                })
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
//                                    intake.setNoodlePower(0);
//                                    outtake.setWheel(0);
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(24.77, 11), Math.toRadians(1.49))
                                .addTemporalMarker(() -> {
//                                    outtake.incrementSlidePos(3);
                                })
                                .splineToConstantHeading(new Vector2d(52, 30), Math.toRadians(0.00))
                                .addTemporalMarker(() -> {
//                                    outtake.setWheel(Outtake.wheelOutDir);
                                })
                                .waitSeconds(0.5)
                                .forward(3)
                                .addTemporalMarker(() -> {
//                                    outtake.setWheel(0);
//                                    outtake.retractSlides();
                                })
                                .build()

                );


//            BufferedImage image = ImageIO.read(new URL("https://i.imgur.com/EOT4Pcj.png"));
//            meepMeep.setBackground(image)
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(myBot)
                .start();

    }

    public static void main(String[] args) {
       traj2();
    }
}