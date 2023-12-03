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

    public static double intakeLH = 1;
    public static double intakeLL= 0;
    public static double intakeRH = 1;
    public static double intakeRL = 0;

    public DcMotorEx noodleMotor;
    // Left/Right when you consider slides to be front of robot
    public Servo intakeLiftL;
    public Servo intakeLiftR;

    public void init(LinearOpMode opmode) {

        noodleMotor = opmode.hardwareMap.get(DcMotorEx.class, "noodleMotor");
        noodleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        noodleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakeLiftL = opmode.hardwareMap.get(Servo.class, "leftIntake");
        intakeLiftR = opmode.hardwareMap.get(Servo.class, "rightIntake");

        setHeight(0);

    }

    public int cycleNoodles() {
        double currentPower = noodleMotor.getPower();
        if(currentPower > 0.1) {
            currentPower = -1;
        } else if(currentPower < -0.1) {
            currentPower = 0;
        } else {
            currentPower = 1;
        }

        noodleMotor.setPower(currentPower);
        return (int) currentPower;

    }

    public void setHeight(double height) {

        height = Range.clip(height, 0, 1);

        intakeLiftL.setPosition(intakeLL + (intakeLH-intakeLL) * height);
        intakeLiftR.setPosition(intakeRL + (intakeRH-intakeRL) * height);

    }

}
