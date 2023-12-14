package org.centennialrobotics.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.centennialrobotics.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Outtake extends Subsystem {

    public static double slideP = 0.005;
    public static double slideF = 0.1;

    public static double pivotHeight = 300;

    public static double pivotFlat = 0.35;
    public static double pivotUp = 0.65;

    public int slidesTarget = -10;

    public static int wheelOutDir = 1;

    public DcMotorEx slideMotorL;
    public DcMotorEx slideMotorR;

    public Servo arm;

    public CRServo wheel;

    private int[] targets = {-10, 350, 450, 650};

    private LinearOpMode opmode;

    public void init(LinearOpMode opmode) {
        this.opmode = opmode;
        arm = opmode.hardwareMap.get(Servo.class, "outtakeServoL");

        setArm(false);

        slideMotorL = opmode.hardwareMap.get(DcMotorEx.class, "slideMotorL");
        slideMotorR = opmode.hardwareMap.get(DcMotorEx.class, "slideMotorR");

        slideMotorR.setDirection(DcMotorEx.Direction.REVERSE);

        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while(slideMotorR.getCurrent(CurrentUnit.AMPS) < 3 && opmode.opModeInInit()) {
            slideMotorR.setPower(-0.4);
            slideMotorL.setPower(-0.4);
        }
        slideMotorR.setPower(0);
        slideMotorL.setPower(0);

        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        wheel = opmode.hardwareMap.get(CRServo.class, "wheel");

    }

    public void incrementSlidePos(int inc) {
        for(int i = 0; i < targets.length; i++) {
            if(targets[i] == slidesTarget) {
                slidesTarget = targets[Range.clip((i+inc), 0, targets.length-1)];
                break;
            }
        }
    }

    public void retractSlides() {
        slidesTarget = -10;
    }

    public void setWheel(double power) {
        wheel.setPower(power);
    }

    public void setArm(boolean out) {
        if(out) {
            arm.setPosition(pivotUp);
        } else {
            arm.setPosition(pivotFlat);
        }
    }


    public void update() {

        int pos = -slideMotorR.getCurrentPosition();
        opmode.telemetry.addData("pos", pos);
        opmode.telemetry.addData("target", slidesTarget);
        double error = slidesTarget - pos;

        double power = Range.clip(error*slideP + slideF, -.25, 1);
        opmode.telemetry.addData("power", power);
        opmode.telemetry.update();

        slideMotorL.setPower(power);
        slideMotorR.setPower(power);

        setArm(pos > pivotHeight && slidesTarget > pivotHeight);
    }

}
