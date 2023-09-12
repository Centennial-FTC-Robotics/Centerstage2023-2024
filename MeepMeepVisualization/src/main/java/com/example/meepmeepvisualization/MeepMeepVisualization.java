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
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, -Math.PI/2))
                                .lineToSplineHeading(new Pose2d(-36, 12, Math.PI/2))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(24, 12, 0))
                                .splineToConstantHeading(new Vector2d(48, 32), 0)
                                .build()
                );

        try {
            BufferedImage image = ImageIO.read(new URL("https://i.imgur.com/g9GR5fM.png"));
            meepMeep.setBackground(image)
                    .setDarkMode(true)
                    .addEntity(myBot)
                    .start();
        } catch (IOException e) {
            System.out.println("Loading background image failed.");

        }
    }
}