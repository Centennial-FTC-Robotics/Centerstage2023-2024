package com.example.meepmeepvisualization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepVisualization {

    public static void traj4() {
        MeepMeep meepMeep = new MeepMeep(900);

        double t = 23.5;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, 3.8, 3.8, 18)
                .followTrajectorySequence(
                        drive -> drive.trajectorySequenceBuilder(new Pose2d(-59.00, 11.00, Math.toRadians(180.00)))
                                .setReversed(true)
                                .splineTo(new Vector2d(22.00, 11.00), Math.toRadians(0.00))
                                .splineTo(new Vector2d(50.48, 41.81), Math.toRadians(0.00))
                                .setReversed(false)
                                .build()












                );




//            BufferedImage image = ImageIO.read(new URL("https://i.imgur.com/EOT4Pcj.png"));
//            meepMeep.setBackground(image)
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(myBot)
                .start();
    }

    public static void traj3() {
        MeepMeep meepMeep = new MeepMeep(700);

        double t = 23.5;

        Globals.Alliance team = Globals.Alliance.RED;
        Globals.StartLoc start = Globals.StartLoc.BACKSTAGE;
        ElementProcessor.PropPositions target = ElementProcessor.PropPositions.MIDDLE;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 3.8, 3.8, 18)
                .followTrajectorySequence(
                        drive -> CRTrajSeqBuilder.init(
                                drive, team, start)
                                .purpleYellowBackstage(target)
                                .returnToIntakeStack(5)
                                .scoreFromStack()
                                .returnToIntakeStack(3)
                                .scoreFromStack()
                                .park(false)
                                .build()



                );




//            BufferedImage image = ImageIO.read(new URL("https://i.imgur.com/EOT4Pcj.png"));
//            meepMeep.setBackground(image)
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(myBot)
                .start();
    }

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
//                                    intake.setHeight(4);
//                                    outtake.setWheel(0);
                                })
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-35.2, 34.76), Math.toRadians(270.00))
                                .setReversed(false)
                                .splineTo(new Vector2d(-42.69, 12.54), Math.toRadians(220.60))
                                .addTemporalMarker(() -> {
//                                    intake.setNoodlePower(0.6);
//                                    outtake.setWheel(-1*Outtake.wheelOutDir);
                                })
                                .splineTo(new Vector2d(-59, 11.05), Math.toRadians(180.00))
                                .setReversed(true)
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
//                                    intake.setNoodlePower(0);
//                                    outtake.setWheel(0);
                                })
                                .splineTo(new Vector2d(24.77, 12.49), Math.toRadians(1.49))
                                .addTemporalMarker(() -> {
//                                    outtake.incrementSlidePos(2);
                                })
                                .splineToConstantHeading(new Vector2d(52, 41), Math.toRadians(0.00))
                                .addTemporalMarker(() -> {
//                                    outtake.setWheel(Outtake.wheelOutDir);
                                })
                                .waitSeconds(1)
                                .setReversed(false)
                                .splineTo(new Vector2d(41.02, 28.63), Math.toRadians(252.35))
                                .addTemporalMarker(() -> {
//                                    outtake.retractSlides();
//                                    outtake.setWheel(0);
//                                    intake.setHeight(3);
//                                    intake.setNoodlePower(0.6);
//                                    outtake.setWheel(-1*Outtake.wheelOutDir);

                                })
                                .splineTo(new Vector2d(13.42, 12.11), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-59, 11.05), Math.toRadians(180.00))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
//                                    intake.setHeight(2);
                                })
                                .waitSeconds(.5)
                                .addTemporalMarker(() -> {
//                                    intake.setNoodlePower(0);
//                                    outtake.setWheel(0);
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(24.77, 12.49), Math.toRadians(1.49))
                                .addTemporalMarker(() -> {
//                                    outtake.incrementSlidePos(2);
                                })
                                .splineToConstantHeading(new Vector2d(52, 30), Math.toRadians(0.00))
                                .addTemporalMarker(() -> {
//                                    outtake.setWheel(Outtake.wheelOutDir);
                                })
                                .waitSeconds(1)
                                .forward(2)
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
       traj3();
    }
}