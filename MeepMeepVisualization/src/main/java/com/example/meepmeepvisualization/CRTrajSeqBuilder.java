package com.example.meepmeepvisualization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class CRTrajSeqBuilder {

    private enum RandomizationPos {
        INNER,
        CENTER,
        OUTER
    }

    public static double BACKDROP_INNER;
    public static double BACKDROP_CENTER;
    public static double BACKDROP_OUTER;
    public static double BACKDROP_DISTANCE;
    public static double STACK_X = -59;
    public static double STACK_Y = 11.05;
    public static double ROBOT_START_Y = 2.5*23.5;


    private TrajectorySequenceBuilder seq;
    private Globals.Alliance team;
    private Globals.StartLoc startLoc;

    public static CRTrajSeqBuilder init(
            DriveShim drive,
            Globals.Alliance team,
            Globals.StartLoc startLoc
    ) {
        return new CRTrajSeqBuilder(drive, team, startLoc);
    }

    public CRTrajSeqBuilder(
            DriveShim drive,
            Globals.Alliance team,
            Globals.StartLoc startLoc
    ) {
        this.team = team;
        this.startLoc = startLoc;

        int mult = (team == Globals.Alliance.RED) ? -1 : 1;
        double startX = (startLoc == Globals.StartLoc.FRONTSTAGE) ? -1.5*23.5 : 0.5*23.5;

        Pose2d startPos = new Pose2d(startX, mult*ROBOT_START_Y, Math.toRadians(mult*270));
        seq = drive.trajectorySequenceBuilder(startPos);
    }

    private RandomizationPos getRandomizationPos(ElementProcessor.PropPositions propPos) {

        RandomizationPos backdropTarget;

        if(propPos == ElementProcessor.PropPositions.MIDDLE) {
            backdropTarget = RandomizationPos.CENTER;
        } else if(
                (propPos == ElementProcessor.PropPositions.LEFT && team == Globals.Alliance.BLUE)
                        || (propPos == ElementProcessor.PropPositions.RIGHT && team == Globals.Alliance.RED)
        ){
            backdropTarget = RandomizationPos.OUTER;
        } else {
            backdropTarget = RandomizationPos.INNER;
        }

        return backdropTarget;
    }

    public CRTrajSeqBuilder purpleDepositFrontstage(
            ElementProcessor.PropPositions propPos, boolean prepStack
    ) {
        RandomizationPos randPos = getRandomizationPos(propPos);

        int mult = (team == Globals.Alliance.RED) ? -1 : 1;

        if(randPos == RandomizationPos.INNER) {

            seq.splineToConstantHeading(new Vector2d(-47.45, 37.48*mult), Math.toRadians(mult*270.00))
                    .addTemporalMarker(() -> {
//                        intake.setHeight(4);
                    });

            if(prepStack) {
                seq.setReversed(true)
                        .splineToConstantHeading(new Vector2d(-35.2, 34.76*mult), Math.toRadians(270.00*mult))
//                    .setReversed(false)
                        .splineTo(new Vector2d(-42.69, 12.54*mult), Math.toRadians(220.60*mult))
                        .addTemporalMarker(() -> {
//                        intake.setNoodlePower(0.6);
//                        outtake.setWheel(-1* Outtake.wheelOutDir);
                        })
                        .splineTo(new Vector2d(STACK_X, mult*STACK_Y), Math.toRadians(180.00));
//                    .setReversed(true);
            }


        } else if(randPos == RandomizationPos.CENTER) {

            seq.splineToLinearHeading(new Pose2d(-39.07, mult*29.83,
                    Math.toRadians(mult*320.00)), Math.toRadians(mult*270.00))
                    .addTemporalMarker(() -> {
//                        intake.setHeight(4);
                    });
            if(prepStack) {
                seq.setReversed(true)
                        .splineToConstantHeading(new Vector2d(-51.63, mult*21.02), Math.toRadians(mult*320.00))
//                                .setReversed(false)

                        .splineTo(new Vector2d(-53.00, STACK_Y*mult), Math.toRadians(mult*180.00))
                        .addTemporalMarker(() -> {
//                        intake.setNoodlePower(0.6);
//                        outtake.setWheel(-1* Outtake.wheelOutDir);
                        })
                        .splineTo(new Vector2d(STACK_X, STACK_Y*mult), Math.toRadians(mult*180.00));
            }

        } else {

            seq.splineTo(new Vector2d(-31.02, 37.71*mult), Math.toRadians(-35.00*mult))
                    .addTemporalMarker(() -> {
//                        intake.setHeight(4);
                    });

            if(prepStack) {
                seq.setReversed(true)
                        .splineToLinearHeading(new Pose2d(STACK_X, mult*STACK_Y,
                                Math.toRadians(mult*180.00)), Math.toRadians(mult*180));
            }

        }


        return this;
    }

    public TrajectorySequence build() {
        return seq.build();
    }



}
