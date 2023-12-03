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

@Config
public class Outtake extends Subsystem {

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    public static int bottomOffset = 50;

    public static double outtakeLL = 0;
    public static double outtakeLH = 1;
    public static double outtakeRL = 0;
    public static double outtakeRH = 1;

    public static int slidesTarget = 0;

    public DcMotorEx slideMotorL;
    public DcMotorEx slideMotorR;

    public Servo armL;
    public Servo armR;

    public CRServo wheel;

    public PIDFController pidfController;

    public void init(LinearOpMode opmode) {

        slideMotorL = opmode.hardwareMap.get(DcMotorEx.class, "slideMotorL");
        slideMotorR = opmode.hardwareMap.get(DcMotorEx.class, "slideMotorR");

        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorL.setDirection(DcMotorEx.Direction.REVERSE);

        pidfController = new PIDFController(P, I, D, F);

        armL = opmode.hardwareMap.get(Servo.class, "outtakeServoL");
        armR = opmode.hardwareMap.get(Servo.class, "outtakeServoR");

        setArm(0);

        wheel = opmode.hardwareMap.get(CRServo.class, "outtakeWheel");

    }

    public void setWheel(double power) {
        wheel.setPower(power);
    }

    public void setArm(double amt) {
        amt = Range.clip(amt, 0, 1);

        armL.setPosition(outtakeLL + (outtakeLH-outtakeLL) * amt);
        armR.setPosition(outtakeRL + (outtakeRH-outtakeRL) * amt);
    }


    public void update() {
        pidfController.setPIDF(P, I, D, F);

        int pos = slideMotorL.getCurrentPosition();

        if(pos < bottomOffset) {
            pidfController.setF(0);
        }

        double power = pidfController.calculate(pos, slidesTarget);

        slideMotorL.setPower(power);
        slideMotorR.setPower(power);
    }

}
