package org.centennialrobotics.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.centennialrobotics.Subsystem;

@Config
public class Climber extends Subsystem {

    public static double hangUp = 0.35;
    public static double hangLow = 0;

    private boolean hangIsUp = false;

    public DcMotorEx hangMotor;
    public Servo hangArm;

    public void init(LinearOpMode opmode) {
        hangMotor = opmode.hardwareMap.get(DcMotorEx.class, "hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hangArm = opmode.hardwareMap.get(Servo.class, "hangServo");
    }

    public void toggleHang() {
        hangIsUp = !hangIsUp;
        if(hangIsUp) {
            hangArm.setPosition(hangUp);
        } else {
            hangArm.setPosition(hangLow);
        }
    }


}
