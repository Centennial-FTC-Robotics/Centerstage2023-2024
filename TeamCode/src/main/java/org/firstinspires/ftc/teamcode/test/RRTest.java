package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.centennialrobotics.subsystems.Intake;
import org.centennialrobotics.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class RRTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Init");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Outtake outtake = new Outtake();
        Intake intake = new Intake();

        outtake.init(this);
        intake.init(this);
        intake.setHeight(Intake.liftLow);

        double t = 23.5;

        telemetry.addData("Status", "Subsystems Inited");
        telemetry.update();

        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d(-1.5*t, 2.5*t, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-47.45, 37.48), Math.toRadians(270.00))
                .addTemporalMarker(() -> {
                                    intake.setHeight(4);
                                    outtake.setWheel(0);
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-35.2, 34.76), Math.toRadians(270.00))
                .setReversed(false)
                .splineTo(new Vector2d(-42.69, 12.54), Math.toRadians(220.60))
                .addTemporalMarker(() -> {
                                    intake.setNoodlePower(0.6);
                                    outtake.setWheel(-1*Outtake.wheelOutDir);
                })
                .splineTo(new Vector2d(-59, 11.05), Math.toRadians(180.00))
                .setReversed(true)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                                    intake.setNoodlePower(0);
                                    outtake.setWheel(0);
                })
                .splineTo(new Vector2d(24.77, 12.49), Math.toRadians(1.49))
                .addTemporalMarker(() -> {
                                    outtake.incrementSlidePos(2);
                })
                .splineToConstantHeading(new Vector2d(52, 41), Math.toRadians(0.00))
                .addTemporalMarker(() -> {
                                    outtake.setWheel(Outtake.wheelOutDir);
                })
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(new Vector2d(41.02, 28.63), Math.toRadians(252.35))
                .addTemporalMarker(() -> {
                                    outtake.retractSlides();
                                    outtake.setWheel(0);
                                    intake.setHeight(3);
                                    intake.setNoodlePower(0.6);
                                    outtake.setWheel(-1*Outtake.wheelOutDir);

                })
                .splineTo(new Vector2d(13.42, 12.11), Math.toRadians(180.00))
                .splineTo(new Vector2d(-59, 11.05), Math.toRadians(180.00))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                                    intake.setHeight(2);
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                                    intake.setNoodlePower(0);
                                    outtake.setWheel(0);
                })
                .setReversed(true)
                .splineTo(new Vector2d(24.77, 12.49), Math.toRadians(1.49))
                .addTemporalMarker(() -> {
                                    outtake.incrementSlidePos(2);
                })
                .splineToConstantHeading(new Vector2d(52, 30), Math.toRadians(0.00))
                .addTemporalMarker(() -> {
                                    outtake.setWheel(Outtake.wheelOutDir);
                })
                .waitSeconds(1)
                .forward(2)
                .addTemporalMarker(() -> {
                                    outtake.setWheel(0);
                                    outtake.retractSlides();
                })
                .build();

        telemetry.addData("Status", "Traj Built");
        telemetry.update();

        drive.setPoseEstimate(ts.start());
        drive.followTrajectorySequenceAsync(ts);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            drive.update();
            outtake.update();
        }
    }
}
