package org.centennialrobotics.util;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.centennialrobotics.processors.ElementProcessor;
import org.centennialrobotics.subsystems.Intake;
import org.centennialrobotics.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Config
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
            SampleMecanumDrive drive,
            Globals.Alliance team,
            Globals.StartLoc startLoc
    ) {
        return new CRTrajSeqBuilder(drive, team, startLoc);
    }

    public CRTrajSeqBuilder(
            SampleMecanumDrive drive,
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
            ElementProcessor.PropPositions propPos,
            Intake intake,
            Outtake outtake
    ) {
        RandomizationPos randPos = getRandomizationPos(propPos);

        int mult = (team == Globals.Alliance.RED) ? -1 : 1;

        if(randPos == RandomizationPos.INNER) {

            seq.splineToConstantHeading(new Vector2d(-47.45, 37.48*mult), Math.toRadians(mult*270.00))
                    .addTemporalMarker(() -> {
                        intake.setHeight(4);
                    })
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(-35.2, 34.76*mult), Math.toRadians(270.00*mult))
//                    .setReversed(false)
                    .splineTo(new Vector2d(-42.69, 12.54*mult), Math.toRadians(220.60*mult))
                    .addTemporalMarker(() -> {
                        intake.setNoodlePower(0.6);
                        outtake.setWheel(-1* Outtake.wheelOutDir);
                    })
                    .splineTo(new Vector2d(STACK_X, mult*STACK_Y), Math.toRadians(180.00));
//                    .setReversed(true);

        } else if(randPos == RandomizationPos.CENTER) {

        } else {

        }


        return this;
    }

    public TrajectorySequence build() {
        return seq.build();
    }



}
