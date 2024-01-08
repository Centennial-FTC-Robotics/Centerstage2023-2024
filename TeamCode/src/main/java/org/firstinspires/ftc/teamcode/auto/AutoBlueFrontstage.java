package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.centennialrobotics.Robot;
import org.centennialrobotics.processors.ElementProcessor;
import org.centennialrobotics.subsystems.Camera;
import org.centennialrobotics.subsystems.Intake;
import org.centennialrobotics.subsystems.Outtake;
import org.centennialrobotics.util.CRTrajSeqBuilder;
import org.centennialrobotics.util.Globals;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutoBlueFrontstage extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Init");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Outtake outtake = new Outtake();
        Intake intake = new Intake();
        Camera cam = new Camera();

        outtake.init(this);
        intake.init(this);
        intake.setHeight(Intake.liftLow);
        cam.init(true,this);


        TrajectorySequence ts = null;
        while(opModeInInit()) {
            ElementProcessor.PropPositions propPos = cam.detectElement();

            ts = CRTrajSeqBuilder.init(
                    drive, Globals.Alliance.BLUE, Globals.StartLoc.FRONTSTAGE)
                    .loadSubsystems(intake, outtake)
                    .purpleDepositFrontstage(propPos, true)
                    .scoreYellowFrontstage(propPos)
                    .returnToIntakeStack(4)
                    .scoreFromStack()
                    .returnToIntakeStack(2)
                    .scoreFromStack()
                    .park(false)
                    .build();

            drive.setPoseEstimate(ts.start());

            telemetry.addData("Detected", propPos.toString());
            telemetry.update();
        }

        waitForStart();
        drive.followTrajectorySequenceAsync(ts);
        while(opModeIsActive()) {
            drive.update();
            outtake.update();
        }
    }

}
