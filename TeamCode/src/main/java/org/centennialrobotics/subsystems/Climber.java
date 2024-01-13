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

    public static double launcherUp = 0.35;

    public static boolean servosEnabled = true;

    public DcMotorEx hangMotor;
    public Servo armBottom;
    public Servo armTop;

    public Servo launcher;
    public Servo launcherLift;

    public boolean launcherLifted = false;

    public void init(LinearOpMode opmode) {
        hangMotor = opmode.hardwareMap.get(DcMotorEx.class, "hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(!Globals.REVERSE_MOTORS)
            hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armBottom = opmode.hardwareMap.get(Servo.class, "bottomHangServo");
        armTop = opmode.hardwareMap.get(Servo.class, "topHangServo");

//        armBottom.setDirection(Servo.Direction.REVERSE);
//        armTop.setDirection(Servo.Direction.REVERSE);

        launcher = opmode.hardwareMap.get(Servo.class, "launcherServo");
        launcherLift = opmode.hardwareMap.get(Servo.class, "launcherLift");

        launcherLift.setDirection(Servo.Direction.REVERSE);

        launcher.setPosition(0);
        launcherLift.setPosition(0);

        down();
    }

    public void launchPlane() {
        launcher.setPosition(1);
    }

    public void setLauncherLift(boolean up) {
        launcherLifted = up;
        if(up) {
            launcherLift.setPosition(launcherUp);
        } else {
            launcherLift.setPosition(0);
        }

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

        armBottom.setPosition(.13);
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
