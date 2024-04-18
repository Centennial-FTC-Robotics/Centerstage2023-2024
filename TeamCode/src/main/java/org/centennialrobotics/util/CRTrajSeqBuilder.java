package org.centennialrobotics.util;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

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

    public static double BACKDROP_INNER = 28.5;
    public static double BACKDROP_CENTER = 36;
    public static double BACKDROP_OUTER = 40;
    public static double BACKDROP_DISTANCE = 55.5;
    public static double STACK_X = -59.5;
    public static double STACK_Y = 11.5;
    public static double OTHER_STACK_Y = 35.5;
    public static double ROBOT_START_Y = 2.5*23.5+2;


    private TrajectorySequenceBuilder seq;
    private Globals.Alliance team;
    private Globals.StartLoc startLoc;
    private SampleMecanumDrive drivebase;

    private Intake intake;
    private Outtake outtake;

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
        this.drivebase = drive;

        int mult = (team == Globals.Alliance.RED) ? -1 : 1;
        double startX = (startLoc == Globals.StartLoc.FRONTSTAGE) ? -1.5*23.5 : 0.5*23.5;

        Pose2d startPos = new Pose2d(startX, mult*ROBOT_START_Y, Math.toRadians(mult*270));
        seq = drive.trajectorySequenceBuilder(startPos);
    }

    public CRTrajSeqBuilder wait(double time) {
        seq.waitSeconds(time);
        return this;
    }

    public CRTrajSeqBuilder loadSubsystems(Intake intake, Outtake outtake) {
        this.intake = intake;
        this.outtake = outtake;
        return this;
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
            ElementProcessor.PropPositions propPos, boolean prepStack, boolean inner
    ) {
        RandomizationPos randPos = getRandomizationPos(propPos);

        int mult = (team == Globals.Alliance.RED) ? -1 : 1;
        double targetStackY = (inner) ? OTHER_STACK_Y : STACK_Y;

        if(randPos == RandomizationPos.INNER) {
            if(!inner) {
                seq.splineToConstantHeading(new Vector2d(-47.45, 37.48*mult), Math.toRadians(mult*270.00))
                        .addTemporalMarker(() -> {
                        intake.setHeight(5);
                        });
            } else {
                seq.splineToLinearHeading(new Pose2d(-42, 35*mult, Math.toRadians(-135*mult)), Math.toRadians(mult*270.00))
                        .addTemporalMarker(() -> {
                        intake.setHeight(5);
                        });
            }


            if(prepStack) {
                if(!inner){
                    seq.setReversed(true)
                            .splineToConstantHeading(new Vector2d(-35.2, 34.76*mult), Math.toRadians(270.00*mult))
//                            .addTemporalMarker(() -> {
//                                intake.setNoodlePower(0.8);
//                                outtake.setWheel(-1* Outtake.wheelOutDir);
//                            })
                            .splineTo(new Vector2d(-42.69, 12.54*mult), Math.toRadians(220.60*mult))
                            .splineTo(new Vector2d(STACK_X-2, mult*targetStackY), Math.toRadians(180.00*mult))
//                            .addTemporalMarker(() -> {
//                                intake.setNoodlePower(0.8);
//                                outtake.setWheel(-1* Outtake.wheelOutDir);
//                            })
//                            .addTemporalMarker(() -> {
//                                intake.setHeight(4);
//                            })
                            .waitSeconds(0.8);
                } else {
                    seq.setReversed(true)

                            .splineToConstantHeading(new Vector2d(-53, 50*mult), Math.toRadians(-135*mult))
                            .lineToLinearHeading(new Pose2d(STACK_X-2, mult*targetStackY, Math.toRadians(180*mult)))
//                            .addTemporalMarker(() -> {
//                                intake.setNoodlePower(0.8);
//                                outtake.setWheel(-1* Outtake.wheelOutDir);
//                            })
//                            .addTemporalMarker(() -> {
//                                intake.setHeight(4);
//                            })
                            .waitSeconds(0.7);
                }

            }


        } else if(randPos == RandomizationPos.CENTER) {

            seq.splineToLinearHeading(new Pose2d(-39.07, mult*28,
                            Math.toRadians(mult*320.00)), Math.toRadians(mult*270.00))
                    .addTemporalMarker(() -> {
                        intake.setHeight(5);
                    });
            if(prepStack) {
                if(!inner) {
                    seq.setReversed(true)
                            .splineToConstantHeading(new Vector2d(-51.63, mult*21.02), Math.toRadians(mult*320.00))

                            .splineTo(new Vector2d(-53.00, targetStackY*mult), Math.toRadians(mult*180.00))
//                        .waitSeconds(0.5)
                            .splineTo(new Vector2d(STACK_X-2, targetStackY*mult), Math.toRadians(mult*180.00))
//                            .addTemporalMarker(() -> {
//                                intake.setNoodlePower(0.8);
//                                outtake.setWheel(-1* Outtake.wheelOutDir);
//                            })
//                            .addTemporalMarker(() -> {
//                                intake.setHeight(4);
//                            })
                            .waitSeconds(0.8);
                } else {
                    seq.setReversed(true)
//                            .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
//                                intake.setNoodlePower(0.8);
//                                outtake.setWheel(-1* Outtake.wheelOutDir);
//                            })
                            .splineToSplineHeading(new Pose2d(-53, 40*mult, Math.toRadians(180*mult)), Math.toRadians(180*mult))
                            .splineToLinearHeading(new Pose2d(STACK_X-2, targetStackY*mult, Math.toRadians(180*mult)), Math.toRadians(mult*180.00))
//                            .addTemporalMarker(() -> {
//                                intake.setHeight(4);
//                            })
                            .waitSeconds(0.3);
                }

            }

        } else {

            seq.splineTo(new Vector2d(-30, 37.71*mult), Math.toRadians(-35.00*mult))
                    .addTemporalMarker(() -> {
                        intake.setHeight(5);
                    });

            if(prepStack) {
                seq.setReversed(true)
                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {
//                            intake.setNoodlePower(0.8);
//                            outtake.setWheel(-1* Outtake.wheelOutDir);
                        })
                        .waitSeconds(0.1) //was 0.5
                        .splineToLinearHeading(new Pose2d(STACK_X-2, mult*targetStackY,
                                Math.toRadians(mult*180.00)), Math.toRadians(mult*180));
//                        .addTemporalMarker(() -> {
//                            intake.setHeight(4);
//                        })
//                        .waitSeconds(0.3);;

            }

        }


        return this;
    }

    public CRTrajSeqBuilder scoreYellowFrontstage(
            ElementProcessor.PropPositions propPos, boolean inner
    ) {
        RandomizationPos randPos = getRandomizationPos(propPos);

        int mult = (team == Globals.Alliance.RED) ? -1 : 1;

        double targetY;

        if(randPos == RandomizationPos.OUTER) {
            targetY = mult * BACKDROP_OUTER;
        } else if(randPos == RandomizationPos.CENTER) {
            targetY = mult * BACKDROP_CENTER;
        } else {
            targetY = mult * BACKDROP_INNER;
        }

        if(!inner) {
            seq.setReversed(false)
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                        intake.setHeight(0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                        intake.setNoodlePower(0);
                        outtake.setWheel(0);
                    })
                    .splineTo(new Vector2d(25.00, STACK_Y*mult), Math.toRadians(0.00))
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        outtake.incrementSlidePos(1);
                    })
                    .splineTo(new Vector2d(BACKDROP_DISTANCE, targetY), Math.toRadians(0.00))
                    .addTemporalMarker(() -> {
                        outtake.setWheel(Outtake.wheelOutDir*0.3);
                    })
                    .resetConstraints()
                    .waitSeconds(1)
                    .setReversed(false);
        } else {
            seq.setReversed(false)
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                        intake.setHeight(0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                        intake.setNoodlePower(0);
                        outtake.setWheel(0);
                    })
                    .splineTo(new Vector2d(-40, 58.75*mult), Math.toRadians(0.00))
                    .splineTo(new Vector2d(25, 58.75*mult), 0)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        outtake.incrementSlidePos(1);
                    })
                    .splineTo(new Vector2d(BACKDROP_DISTANCE, targetY), Math.toRadians(0.00))
                    .addTemporalMarker(() -> {
                        outtake.setWheel(Outtake.wheelOutDir*0.3);
                    })
                    .resetConstraints()
                    .waitSeconds(1)
                    .setReversed(false);
        }

        return this;
    }

    public CRTrajSeqBuilder purpleYellowBackstage(
            ElementProcessor.PropPositions propPos) {

        RandomizationPos randPos = getRandomizationPos(propPos);
        int mult = (team == Globals.Alliance.RED) ? -1 : 1;
        double targetY;

        if(randPos == RandomizationPos.OUTER) {
            targetY = mult * BACKDROP_OUTER+2;
        } else if(randPos == RandomizationPos.CENTER) {
            targetY = mult * BACKDROP_CENTER;
        } else {
            targetY = mult * BACKDROP_INNER-2;
        }

        if(randPos == RandomizationPos.INNER) {
            seq.splineTo(new Vector2d(8.0, mult*35.95), Math.toRadians(mult*200.00));
        } else if(randPos == RandomizationPos.CENTER) {
            seq.splineToLinearHeading(new Pose2d(15.74, mult*29.5,
                    Math.toRadians(245.00*mult)), Math.toRadians(270.00*mult));
        } else {
            seq.splineToLinearHeading(new Pose2d(
                            30.25, mult*31.76, Math.toRadians(mult*180.00)),
                    Math.toRadians(mult*225.00));
        }

        seq.addTemporalMarker(() -> {
                    intake.setHeight(5);
                })
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    outtake.incrementSlidePos(1);
                })
                .splineTo(new Vector2d(BACKDROP_DISTANCE+1, targetY), 0)
                .addTemporalMarker(() -> {
                    outtake.setWheel(0.4*Outtake.wheelOutDir);
                })
                .waitSeconds(.3)
                .setReversed(false);


        return this;

    }

    public CRTrajSeqBuilder scoreFromStack(boolean inner) {

        int mult = (team == Globals.Alliance.RED) ? -1 : 1;
        double backdropY = (inner) ? BACKDROP_OUTER : BACKDROP_INNER;

        if(!inner) {
            seq.setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                        intake.setHeight(0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                        intake.setNoodlePower(-0.8);
                        outtake.setWheel(0);
                    })
                    .splineTo(new Vector2d(25.00, STACK_Y*mult), Math.toRadians(0.00))
                    .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                        if(CRPoseCalc.distance(
                                drivebase.getPoseEstimate(),
                                new Pose2d(25, STACK_Y*mult)) < 24) {
                            outtake.incrementSlidePos(3);
                        }
                        intake.setNoodlePower(0);
                        intake.setBumperUp(true);
                    })
                    .splineTo(new Vector2d(BACKDROP_DISTANCE, mult*backdropY), Math.toRadians(0.00))
                    .addTemporalMarker(() -> {
                        outtake.setWheel(Outtake.wheelOutDir);
                    })
                    .waitSeconds(0.4)
                    .setReversed(false);
        } else {
            seq.setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                        intake.setHeight(0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                        intake.setNoodlePower(-0.8);
                        outtake.setWheel(0);
                    })
                    .splineTo(new Vector2d(-40, 58.75*mult), Math.toRadians(0.00))
                    .splineTo(new Vector2d(25, 58.75*mult), 0)
                    .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                        outtake.incrementSlidePos(3);
                    })
                    .splineTo(new Vector2d(BACKDROP_DISTANCE, mult*backdropY), Math.toRadians(0.00))
                    .addTemporalMarker(() -> {
                        outtake.setWheel(Outtake.wheelOutDir);
                    })
                    .waitSeconds(0.4)
                    .setReversed(false);
        }


        return this;
    }

    public CRTrajSeqBuilder returnToIntakeStack(
            int oneHigherIntakeHeight, boolean inner
    ) {

        int mult = (team == Globals.Alliance.RED) ? -1 : 1;

        if(!inner) {
            seq.setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        outtake.retractSlides();
                        outtake.setWheel(0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                        intake.setHeight(oneHigherIntakeHeight);
                        intake.setBumperUp(false);
                        intake.setNoodlePower(0.8);
                        outtake.setWheel(-1* Outtake.wheelOutDir);
                    })
                    .splineTo(new Vector2d(25.00, STACK_Y*mult), Math.toRadians(mult*180))
                    .splineTo(new Vector2d(STACK_X-0.5, STACK_Y*mult), Math.toRadians(mult*180))
                    .waitSeconds(0.05)
                    .addTemporalMarker(() -> {
                        intake.setHeight(oneHigherIntakeHeight-1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                        intake.setHeight(oneHigherIntakeHeight-2);
                    })
                    .waitSeconds(.7);
        } else {
            seq.setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        outtake.retractSlides();
                        outtake.setWheel(0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                        intake.setHeight(oneHigherIntakeHeight);
                        intake.setNoodlePower(0.8);
                        outtake.setWheel(-1* Outtake.wheelOutDir);
                    })
                    .splineTo(new Vector2d(25.00, 58.75*mult), Math.toRadians(mult*180))
                    .splineTo(new Vector2d(-40, 58.75*mult), Math.toRadians(mult*180))
                    .splineTo(new Vector2d(STACK_X-0.5, mult*OTHER_STACK_Y), Math.toRadians(180))
                    .waitSeconds(0.05)
                    .addTemporalMarker(() -> {
                        intake.setHeight(oneHigherIntakeHeight-1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                        intake.setHeight(oneHigherIntakeHeight-2);
                    })
                    .waitSeconds(.7);
        }


        return this;
    }

    public CRTrajSeqBuilder park(boolean full) {
        int mult = (team == Globals.Alliance.RED) ? -1 : 1;

        if(!full) {
            seq.setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        outtake.setWheel(0);
                        outtake.retractSlides();
                    })
                    .forward(6);
            return this;
        }

        seq.setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    outtake.retractSlides();
                    outtake.setWheel(0);
                })
                .splineToConstantHeading(new Vector2d(BACKDROP_DISTANCE, mult*60), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(62, 62*mult), Math.toRadians(1.21*mult));

        return this;
    }

    public TrajectorySequence build() {
        return seq.build();
    }



}
