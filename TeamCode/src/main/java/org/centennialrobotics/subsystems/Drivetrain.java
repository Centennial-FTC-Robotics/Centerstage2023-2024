package org.centennialrobotics.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.centennialrobotics.Subsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

@Config
public class Drivetrain extends Subsystem {

    public final double DRIVE_MOTOR_RPM = 340;
    public final double DRIVE_MOTOR_CPR = 537.6;


    public SampleMecanumDrive drivebase;

    // static pose storage that persists between opmodes
    // update this at the end of auto to transfer the pose to teleop
    public static Pose2d currentPose = new Pose2d();

    public LinearOpMode opmode;

    public double targetHeading = -1000000;

    public static double headingP = 0.005;

    public void init(LinearOpMode opmode) {
        this.opmode = opmode;

        drivebase = new SampleMecanumDrive(opmode.hardwareMap);

    }

    public void setRotationLock(boolean activate) {
        if(activate && targetHeading < -999999) {
            targetHeading = drivebase.getPoseEstimate().getHeading();
        } else if(!activate) {
            targetHeading = -1000000;
        }
    }


    public void drive(double forward, double strafe, double turn) {

        if(targetHeading > -999999) {
            double error = drivebase.getPoseEstimate().getHeading() - targetHeading;
            turn = headingP * error;
        }

        drivebase.setWeightedDrivePower(
                new Pose2d(
                        forward,
                        -strafe,
                        -turn
                )
        );

    }

}
