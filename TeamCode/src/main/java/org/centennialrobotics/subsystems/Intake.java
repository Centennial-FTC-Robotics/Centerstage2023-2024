package org.centennialrobotics.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.centennialrobotics.Subsystem;
import org.centennialrobotics.util.Globals;

@Config
public class Intake extends Subsystem {

    public static double intakePower = 0.6;

    public static double liftLow = 1;
    public static double liftMid = .5;
    public static double liftHigh = 0;

    public static double bumperDownL = 0.125;
    public static double bumperUpL = 0.7;
    public static double bumperDownR = 0.85;
    public static double bumperUpR = 0.5;

//    public double[] heights = {0.63, 0.58, 0.52, 0.465, 0.4, 0};
    public double[] heights = {0.24, 0.35, 0.45, 0.54, 0.64, 0.9};
    public int currentHeight = 0;

    // high: 0
    // 5th:
    // 4th: 0.465
    // 3rd: 0.52
    // 2nd: .58
    // Ground: 0.64



    public DcMotorEx noodleMotor;
    // Left/Right when you consider slides to be front of robot
    public Servo intakeLift;
    public Servo bumperL;
    public Servo bumperR;
    public CRServo roller;

    public void init(LinearOpMode opmode) {

        noodleMotor = opmode.hardwareMap.get(DcMotorEx.class, "noodleMotor");
        noodleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        noodleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if(!Globals.REVERSE_MOTORS)
            noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakeLift = opmode.hardwareMap.get(Servo.class, "leftIntake");
        intakeLift.setDirection(Servo.Direction.REVERSE);

        roller = opmode.hardwareMap.get(CRServo.class, "intakeRoller");

        setHeight(currentHeight);

        bumperL = opmode.hardwareMap.get(Servo.class, "leftBlocker");
        bumperR = opmode.hardwareMap.get(Servo.class, "rightBlocker");
        setBumperUp(true);

    }

    public void setBumperUp(boolean up) {
        if(up) {
            bumperL.setPosition(bumperUpL);
            bumperR.setPosition(bumperUpR);
        } else {
            bumperL.setPosition(bumperDownL);
            bumperR.setPosition(bumperDownR);
        }
    }

    public void expelOne() throws InterruptedException {
        noodleMotor.setPower(-0.17);
        Thread.sleep(1000);
        noodleMotor.setPower(0);
    }

    public void setNoodlePower(double power) {
        noodleMotor.setPower(Range.clip(power, -1, 1));
        roller.setPower(Math.signum(power));
    }

    public int cycleNoodles() {
        double currentPower = noodleMotor.getPower();
        if(currentPower > 0.1) {
            currentPower = -1*intakePower;
        } else if(currentPower < -0.1) {
            currentPower = 0;
        } else {
            currentPower = 1*intakePower;
        }

        noodleMotor.setPower(currentPower);
        return (int) currentPower;

    }

    public void incHeight(int inc) {
        setHeight(currentHeight+inc);
    }

    public void setHeight(int heightLevel) {
        if(heightLevel < 0 || heightLevel >= heights.length) {
            return;
        }
        currentHeight = heightLevel;
        setHeight(heights[currentHeight]+0.05);
    }

    public void setHeight(double height) {

//        height = Range.clip(height*(liftHigh-liftLow)+liftLow, 0, 1);

        intakeLift.setPosition(height);


    }

}
