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

    public static double hangUnlocked = 0.6;
    public static double hangLocked = 0.00;

    public static double launcherUp = 0.35;

    public DcMotorEx hangMotor;
    public Servo hangLock;

    public Servo launcher;
    public Servo launcherLift;

    public boolean launcherLifted = false;
    public boolean isHangLocked = true;

    public void init(LinearOpMode opmode) {
        hangMotor = opmode.hardwareMap.get(DcMotorEx.class, "hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(!Globals.REVERSE_MOTORS)
            hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hangLock = opmode.hardwareMap.get(Servo.class, "hangLock");

//        armBottom.setDirection(Servo.Direction.REVERSE);
//        armTop.setDirection(Servo.Direction.REVERSE);

        setHangLock(true);

        launcher = opmode.hardwareMap.get(Servo.class, "launcherServo");
        launcherLift = opmode.hardwareMap.get(Servo.class, "launcherLift");

        launcherLift.setDirection(Servo.Direction.REVERSE);

        launcher.setPosition(0);
        launcherLift.setPosition(0);

    }

    public void launchPlane() {
        launcher.setPosition(0.15);
    }

    public void setHangLock(boolean locked) {
        if(locked) {
            hangLock.setPosition(hangLocked);
        } else {
            hangLock.setPosition(hangUnlocked);
        }
        isHangLocked = locked;
    }

    public void setLauncherLift(boolean up) {
        launcherLifted = up;
        if(up) {
            launcherLift.setPosition(launcherUp);
        } else {
            launcherLift.setPosition(0);
        }

    }

}
