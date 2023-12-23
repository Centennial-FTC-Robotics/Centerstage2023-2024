package org.centennialrobotics.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.centennialrobotics.Subsystem;

@Config
public class Intake extends Subsystem {

    public static double intakePower = 0.6;

    public static double liftLow = 1;
    public static double liftHigh = 0;

    public DcMotorEx noodleMotor;
    // Left/Right when you consider slides to be front of robot
    public Servo intakeLift;

    public void init(LinearOpMode opmode) {

        noodleMotor = opmode.hardwareMap.get(DcMotorEx.class, "noodleMotor");
        noodleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        noodleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakeLift = opmode.hardwareMap.get(Servo.class, "leftIntake");
        intakeLift.setDirection(Servo.Direction.REVERSE);

        setHeight(0);

    }

    public void expelOne() throws InterruptedException {
        noodleMotor.setPower(-0.17);
        Thread.sleep(1000);
        noodleMotor.setPower(0);
    }

    public void setNoodlePower(double power) {

        noodleMotor.setPower(Range.clip(power, -1, 1));
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

    public void setHeight(double height) {

        height = Range.clip(height*(liftHigh-liftLow)+liftLow, 0, 1);

        intakeLift.setPosition(height);

    }

}
