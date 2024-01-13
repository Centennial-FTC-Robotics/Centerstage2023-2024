package org.centennialrobotics.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.centennialrobotics.Subsystem;
import org.centennialrobotics.util.Globals;

@Config
public class Climber extends Subsystem {

    public static double hangUp = 0.35;
    public static double hangLow = 0;

    public static boolean servosEnabled = true;

    public DcMotorEx hangMotor;
    public Servo armBottom;
    public Servo armTop;

    public CRServo launcher;



    public void init(LinearOpMode opmode) {
        hangMotor = opmode.hardwareMap.get(DcMotorEx.class, "hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(!Globals.REVERSE_MOTORS)
            hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armBottom = opmode.hardwareMap.get(Servo.class, "bottomHangServo");
        armTop = opmode.hardwareMap.get(Servo.class, "topHangServo");

        launcher = opmode.hardwareMap.get(CRServo.class, "launcherServo");

        down();
    }

    public void up() {
        armBottom.getController().pwmEnable();
        armTop.getController().pwmEnable();

        armBottom.setPosition(.5);
        armTop.setPosition(0);
    }

    public void down() {
        armBottom.getController().pwmEnable();
        armTop.getController().pwmEnable();

        armBottom.setPosition(.16);
        armTop.setPosition(1);
    }

    public void left() {
        armBottom.getController().pwmEnable();
        armTop.getController().pwmEnable();

        armBottom.setPosition(.4);
        armTop.setPosition(.2);
    }

    public void right() {
        armBottom.getController().pwmEnable();
        armTop.getController().pwmEnable();

        armBottom.setPosition(.5);
        armTop.setPosition(.25);
    }

    public void setServoEnabled(boolean enabled) {

        if(enabled) {
            armBottom.getController().pwmEnable();
            armTop.getController().pwmEnable();
        } else {
            armBottom.getController().pwmDisable();
            armTop.getController().pwmDisable();
        }
    }


}
