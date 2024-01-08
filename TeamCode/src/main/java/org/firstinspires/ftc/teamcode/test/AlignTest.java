package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.centennialrobotics.Robot;
import org.centennialrobotics.subsystems.Drivetrain;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class AlignTest extends LinearOpMode {

    public static double P = 0.1;
    public static double limit = 0.5;
    public static double forwardMultiplier = 1;
    public static double sideMultiplier = 1;
    double power = 0;
    double leftDist = 0;
    double rightDist = 0;

    public void runOpMode() throws InterruptedException{

        Drivetrain dt = new Drivetrain();
        dt.init(this);

        DistanceSensor distanceLeft =
                hardwareMap.get(DistanceSensor.class, "distanceLeft");
        DistanceSensor distanceRight =
                hardwareMap.get(DistanceSensor.class, "distanceRight");

        GamepadEx drivePad = new GamepadEx(gamepad1);
        GamepadEx toolPad = new GamepadEx(gamepad2);

        waitForStart();
        while(opModeIsActive()){
            drivePad.readButtons();
            toolPad.readButtons();

            if(drivePad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {

                //leftDist = distanceLeft.getDistance(DistanceUnit.CM);
                //rightDist = distanceRight.getDistance(DistanceUnit.CM);

                double error = leftDist - rightDist;
                if(Math.abs(error) < 2 || leftDist > 30 || rightDist > 30){
                    error = 0;
                }

                power = drivePad.getRightX();//Range.clip(P*error, -limit, limit);
                if(Math.abs(drivePad.getLeftY()) >= Math.abs(drivePad.getLeftX())) {
                    forwardMultiplier = -0.4;
                    sideMultiplier = 0;
                } else {
                    forwardMultiplier = 0;
                    sideMultiplier = -0.4;
                }
            } else {
                forwardMultiplier = 1;
                sideMultiplier = 1;
                power = drivePad.getRightX();
            }

            dt.drive(
                    forwardMultiplier*drivePad.getLeftY(),
                    sideMultiplier*drivePad.getLeftX(),
                    power
            );
            telemetry.addData("left", leftDist);
            telemetry.addData("right", rightDist);
            telemetry.update();

        }
    }

}
