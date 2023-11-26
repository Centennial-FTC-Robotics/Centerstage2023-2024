package org.centennialrobotics.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.centennialrobotics.Subsystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;

import java.util.List;

public class Drivetrain extends Subsystem {

    public final double DRIVE_MOTOR_RPM = 340;
    public final double DRIVE_MOTOR_CPR = 537.6;

    public MecanumDrive drivebase;
    public StandardTrackingWheelLocalizer localizer;

    // static pose storage that persists between opmodes
    // update this at the end of auto to transfer the pose to teleop
    public static Pose2d currentPose = new Pose2d();

    public void init(LinearOpMode opmode) {

        Motor frontLeft =
                new Motor(opmode.hardwareMap, "frontLeft", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        Motor frontRight =
                new Motor(opmode.hardwareMap, "frontRight", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        Motor backLeft =
                new Motor(opmode.hardwareMap, "backLeft", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        Motor backRight =
                new Motor(opmode.hardwareMap, "backRight", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        drivebase = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

//        /////////// add the localizer here once encoders are set up
//        localizer = new StandardTrackingWheelLocalizer(opmode.hardwareMap, );

    }

    public void drive(double forward, double strafe, double turn, boolean fieldCentric) {
        if(fieldCentric) {
            // maybe use odo for heading or average odo and imu? faster cycles maybe?
            drivebase.driveFieldCentric(strafe, forward, turn, robot.imu.revIMU.getHeading());
        } else {
            drivebase.driveRobotCentric(strafe, forward, turn);
        }
    }

    public void teleOpUpdate(Gamepad gamepad1, Gamepad gamepad2){
        drivebase.driveRobotCentric(
                (gamepad1.left_stick_x),
                (gamepad1.left_stick_y),
                (gamepad1.right_stick_x),
                false
        );
    }


}
