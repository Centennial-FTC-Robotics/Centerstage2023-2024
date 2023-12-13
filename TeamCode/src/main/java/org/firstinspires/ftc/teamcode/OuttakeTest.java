package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.util.RegressionUtil;

@TeleOp
@Config
public class OuttakeTest extends LinearOpMode {

    public static double P = 0.005;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0.1;
    public static double target = 0;


    public void runOpMode() {

        DcMotorEx slideL = hardwareMap.get(DcMotorEx.class, "slideMotorL");
        DcMotorEx slideR = hardwareMap.get(DcMotorEx.class, "slideMotorR");

        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        slideR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {

            int pos = -slideR.getCurrentPosition();
            double error = target - pos;

            double power = Range.clip(error*P + F, -.2, 1);

            slideL.setPower(power);
            slideR.setPower(power);

            telemetry.addData("pos", pos);
            telemetry.addData("leftMotorCurrent", slideL.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightMotorCurrent", slideR.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }

    }

}
