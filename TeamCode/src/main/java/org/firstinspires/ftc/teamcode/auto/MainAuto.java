package org.firstinspires.ftc.teamcode.auto;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.centennialrobotics.processors.ElementProcessor;
import org.centennialrobotics.subsystems.Camera;
import org.centennialrobotics.subsystems.Intake;
import org.centennialrobotics.subsystems.Outtake;
import org.centennialrobotics.util.CRMenu;
import org.centennialrobotics.util.CRTrajSeqBuilder;
import org.centennialrobotics.util.Globals;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Photon
@Autonomous
public class MainAuto extends LinearOpMode {

    SampleMecanumDrive drive;
    Outtake outtake;
    Intake intake;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Init");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);
        outtake = new Outtake();
        intake = new Intake();
        Camera cam = new Camera();

        outtake.init(this);
        intake.init(this);
        intake.setHeight(Intake.liftLow);

//        Servo armBottom = hardwareMap.get(Servo.class, "bottomHangServo");
//        Servo armTop = hardwareMap.get(Servo.class, "topHangServo");
//        armBottom.setPosition(0.5);
//        armTop.setPosition(0.5);

        Globals.Alliance team = new CRMenu<>(
                Globals.Alliance.BLUE, Globals.Alliance.RED, null, null).get(this);

        cam.init(team == Globals.Alliance.BLUE,this);

        Globals.StartLoc startLoc = new CRMenu<>(
                Globals.StartLoc.FRONTSTAGE, Globals.StartLoc.BACKSTAGE, null, null).get(this);

        String path = null;
        if(startLoc == Globals.StartLoc.FRONTSTAGE) {

            path = new CRMenu<>("1+0", "2+1", "2+3", "2+5").get(this);

        } else if(startLoc == Globals.StartLoc.BACKSTAGE){

            path = new CRMenu<>("2+0", "2+2", "2+4", null).get(this);

        }

        boolean inner = new CRMenu<>(true, false, null, null).get(this);


        TrajectorySequence ts = null;
        while(opModeInInit()) {
            ElementProcessor.PropPositions propPos = cam.detectElement();

            ts =getPath(team, startLoc, propPos, inner, path);

            drive.setPoseEstimate(ts.start());

            telemetry.addData("Detected", propPos.toString());
            telemetry.update();
        }

        waitForStart();
//        armBottom.setPosition(.13);
//        armTop.setPosition(1);
        drive.followTrajectorySequenceAsync(ts);

        double lastFrame = 0;

        while(opModeIsActive()) {
            drive.update();
            outtake.update();

            double currFrame = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (currFrame - lastFrame));
            lastFrame = currFrame;
            telemetry.update();
        }
    }

    public TrajectorySequence getPath(
            Globals.Alliance team,
            Globals.StartLoc startLoc,
            ElementProcessor.PropPositions propPos,
            boolean inner,
            String path) {

        if(startLoc == Globals.StartLoc.FRONTSTAGE) {
            if(path.equals("2+3")) {
                return CRTrajSeqBuilder.init(
                                drive, team, startLoc)
                        .loadSubsystems(intake, outtake)
                        .wait(8.)
                        .purpleDepositFrontstage(propPos, true, inner)
                        .scoreYellowFrontstage(propPos, inner)
                        .returnToIntakeStack(4, inner)
                        .scoreFromStack(inner)
                        .park(false)
                        .build();
            } else if(path.equals("2+5")) {
                return CRTrajSeqBuilder.init(
                                drive, team, startLoc)
                        .loadSubsystems(intake, outtake)
                        .purpleDepositFrontstage(propPos, true, inner)
                        .scoreYellowFrontstage(propPos, inner)
                        .returnToIntakeStack(4, inner)
                        .scoreFromStack(inner)
                        .returnToIntakeStack(2, inner)
                        .scoreFromStack(inner)
                        .park(false)
                        .build();
            } else if(path.equals("1+0")) {
                return CRTrajSeqBuilder.init(
                                drive, team, startLoc)
                        .loadSubsystems(intake, outtake)
                        .purpleDepositFrontstage(propPos, false, inner)
//                        .park(false)
                        .build();
            } else if(path.equals("2+1")) {
                return CRTrajSeqBuilder.init(
                        drive, team, startLoc)
                        .loadSubsystems(intake, outtake)
                        .wait(14.)
                        .purpleDepositFrontstage(propPos, true, inner)
                        .scoreYellowFrontstage(propPos, inner)
                        .park(false)
                        .build();

            }
        } else if(startLoc == Globals.StartLoc.BACKSTAGE) {

            if (path.equals("2+0")) {
                return CRTrajSeqBuilder.init(drive, team, startLoc)
                        .loadSubsystems(intake, outtake)
                        .purpleYellowBackstage(propPos)
                        .park(true)
                        .build();

            } else if(path.equals("2+2")) {
                return CRTrajSeqBuilder.init(drive, team, startLoc)
                        .loadSubsystems(intake, outtake)
                        .purpleYellowBackstage(propPos)
                        .returnToIntakeStack(5, inner)
                        .scoreFromStack(inner)
                        .park(false)
                        .build();

            } else if(path.equals("2+4")) {
                return CRTrajSeqBuilder.init(drive, team, startLoc)
                        .loadSubsystems(intake, outtake)
                        .purpleYellowBackstage(propPos)
                        .returnToIntakeStack(5, inner)
                        .scoreFromStack(inner)
                        .returnToIntakeStack(3, inner)
                        .scoreFromStack(inner)
                        .park(false)
                        .build();
            }

        }

        return null;
    }
}
