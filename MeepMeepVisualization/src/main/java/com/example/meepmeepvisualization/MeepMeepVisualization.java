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

    public static void traj1() {
        MeepMeep meepMeep = new MeepMeep(700);

        double tile = 23.5;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, 3.8, 3.8, 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-1.5*tile, 2.5*tile, 3*Math.PI/2))
                                .lineToSplineHeading(new Pose2d(-1.5*tile, 1.25*tile, 0))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-35, 3))
                                .lineToConstantHeading(new Vector2d(18, 3))
                                .splineTo(new Vector2d(48, 36), 0)
                                .build()
                );

        try {
            BufferedImage image = ImageIO.read(new URL("https://i.imgur.com/EOT4Pcj.png"));
            meepMeep.setBackground(image)
//            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(false)
                    .addEntity(myBot)
                    .start();
        } catch (IOException e) {
            System.out.println("Loading background image failed.");

        }
    }

    public static void traj2() {
        MeepMeep meepMeep = new MeepMeep(700);

        double t = 23.5;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, 3.8, 3.8, 18)
                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-1.5*t, 2.5*t, Math.toRadians(-90)))
//                                .splineToSplineHeading(new Pose2d(-1.9*t, 1.6*t, Math.toRadians(-100)), Math.toRadians(-100))
//                                .waitSeconds(0.5)
//                                .splineToSplineHeading(new Pose2d(-1.5*t, 1.5*t, Math.toRadians(-180)), Math.toRadians(-90))
//                                .splineToConstantHeading(new Vector2d(-2.5*t, 0.5*t), Math.toRadians(180))
//                                .build()
                        drive.trajectorySequenceBuilder(new Pose2d(-1.5*t, 2.5*t, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(-47.45, 37.48), Math.toRadians(270.00))
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-35.46, 36.76), Math.toRadians(270.00))
                                .setReversed(false)
                                .splineTo(new Vector2d(-43.69, 16.54), Math.toRadians(220.60))
                                .splineTo(new Vector2d(-62.03, 11.05), Math.toRadians(180.00))
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(24.77, 12.49), Math.toRadians(1.49))
                                .splineTo(new Vector2d(49.47, 41.67), Math.toRadians(0.00))
                                .setReversed(false)
                                .waitSeconds(.5)

                                .build()
                );

        try {
            BufferedImage image = ImageIO.read(new URL("https://i.imgur.com/EOT4Pcj.png"));
            meepMeep.setBackground(image)
//            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(false)
                    .addEntity(myBot)
                    .start();
        } catch (IOException e) {
            System.out.println("Loading background image failed.");

        }
    }

    public static void main(String[] args) {
       traj2();
    }
}