package org.centennialrobotics.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.centennialrobotics.util.Globals;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Outtake extends Subsystem {

    public static double slideP = 0.005;
    public static double slideI = 0.0001;
    public static double slideD = 0;
    public static double slideF = 0.1;
    public static int errorThreshold = 5;

    public static double pivotHeight = 150;

    public static double pivotFlat = 0.38;
    public static double pivotUp = 0.7;

    public static double maxDownSpeed = 0.35;

    public int slidesTarget = 0;

    public int errorSum = 0;
    public int lastError = 0;
    public long lastTime = 0;

    public static int wheelOutDir = 1;

    public DcMotorEx slideMotorL;
    public DcMotorEx slideMotorR;

    public Servo arm;

    public CRServo wheel;

    private int[] targets = {0, 350-25, 420-25, 500-25, 580-25, 640-25};
//    public static int slideOffset = 0;

    private LinearOpMode opmode;

    public void init(LinearOpMode opmode) throws InterruptedException {
        this.opmode = opmode;
        arm = opmode.hardwareMap.get(Servo.class, "outtakeServoL");

        setArm(false);

        slideMotorL = opmode.hardwareMap.get(DcMotorEx.class, "slideMotorL");
        slideMotorR = opmode.hardwareMap.get(DcMotorEx.class, "slideMotorR");

        if(!Globals.REVERSE_MOTORS) {
            slideMotorR.setDirection(DcMotorEx.Direction.REVERSE);
        } else slideMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while(slideMotorR.getCurrent(CurrentUnit.AMPS) < 3 && opmode.opModeInInit()) {
            slideMotorR.setPower(-0.4);
            slideMotorL.setPower(-0.4);
        }
        slideMotorR.setPower(0);
        slideMotorL.setPower(0);

        Thread.sleep(700);

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
        slidesTarget = 0;
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

        long t = System.currentTimeMillis();

        if(lastTime == 0) {
            lastTime = t-1;
        }


        int pos = -slideMotorR.getCurrentPosition();
        double error = slidesTarget - pos;

        double speed = (double)(error-lastError)/(double)(t-lastTime);

        if(Math.abs(error) < errorThreshold) {
            errorSum = 0;
        } else {
            errorSum += error;
        }

        Telemetry tel =  FtcDashboard.getInstance().getTelemetry();
        tel.addData("target", slidesTarget);
        tel.addData("pos", pos);
        tel.update();

        double power = Range.clip(error*slideP + errorSum*slideI + +speed*slideD + slideF,
                -maxDownSpeed, 1);

        slideMotorL.setPower(power);
        slideMotorR.setPower(power);

        setArm(pos > pivotHeight && slidesTarget > pivotHeight);

        lastTime = t;
    }

}
