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
        MeepMeep meepMeep = new MeepMeep(700);

        double tile = 23.75;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
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
}