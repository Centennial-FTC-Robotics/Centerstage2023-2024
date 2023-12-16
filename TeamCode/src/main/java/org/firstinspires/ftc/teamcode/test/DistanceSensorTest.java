package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.centennialrobotics.subsystems.Drivetrain;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class DistanceSensorTest extends LinearOpMode {

    public static double P = 0;
    public static double limit = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        DistanceSensor distanceLeft =
                hardwareMap.get(DistanceSensor.class, "distanceLeft");
        DistanceSensor distanceRight =
                hardwareMap.get(DistanceSensor.class, "distanceRight");

        Drivetrain dt = new Drivetrain();
        dt.init(this);

        waitForStart();
        while(opModeIsActive()) {

            double leftDist = distanceLeft.getDistance(DistanceUnit.CM);
            double rightDist = distanceRight.getDistance(DistanceUnit.CM);

            double error = leftDist - rightDist;

            telemetry.addData("left", leftDist);
            telemetry.addData("right", rightDist);
            telemetry.update();

            double power = Range.clip(P*error, -limit, limit);

            dt.drive(0, 0, power, false);

        }

    }
}
