package org.centennialrobotics.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.centennialrobotics.Subsystem;


public class Intake extends Subsystem {

    public static double intakeDown = 1;
    public static double intakeUp= 0;

    public static double motorPower = 1;

    public DcMotorEx noodleMotor;
    // Left/Right when you consider slides to be front of robot
    public Servo intakeLift;

    public void init(LinearOpMode opmode) {

        noodleMotor = opmode.hardwareMap.get(DcMotorEx.class, "noodleMotor");
        noodleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        noodleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakeLift = opmode.hardwareMap.get(Servo.class, "intakeLift");

        setHeight(0);

    }

    public int cycleNoodles() {
        double currentPower = noodleMotor.getPower();
        if(currentPower > 0.1) {
            currentPower = -1*motorPower;
        } else if(currentPower < -0.1) {
            currentPower = 0;
        } else {
            currentPower = 1*motorPower;
        }

        noodleMotor.setPower(currentPower);
        return (int) currentPower;

    }

    public void setHeight(double height) {

        height = Range.clip(height, 0, 1);

        intakeLift.setPosition(intakeDown + (intakeUp-intakeDown) * height);

    }

}
