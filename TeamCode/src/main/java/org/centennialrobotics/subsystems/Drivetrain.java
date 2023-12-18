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

import java.util.List;

@Config
public class Drivetrain extends Subsystem {

    public final double DRIVE_MOTOR_RPM = 340;
    public final double DRIVE_MOTOR_CPR = 537.6;

    public MecanumDrive drivebase;
    public StandardTrackingWheelLocalizer localizer;

    // static pose storage that persists between opmodes
    // update this at the end of auto to transfer the pose to teleop
    public static Pose2d currentPose = new Pose2d();

    public Motor.Encoder parallelLeft;
    public Motor.Encoder parallelRight;
    public Motor.Encoder perpendicularBack;

    public LinearOpMode opmode;

    public double targetHeading = 0;

    public static double headingP = 0.05;
    public static double headingLimit = 0.4;

    public double multiplier = 1;

    public void init(LinearOpMode opmode) {
        this.opmode = opmode;

        Motor frontLeft =
                new Motor(opmode.hardwareMap, "frontLeft", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        Motor frontRight =
                new Motor(opmode.hardwareMap, "frontRight", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        Motor backLeft =
                new Motor(opmode.hardwareMap, "backLeft", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        Motor backRight =
                new Motor(opmode.hardwareMap, "backRight", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        drivebase = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // change these and REVERSE some of them if necessary
        parallelLeft = frontRight.encoder; //1
        parallelRight = backRight.encoder; //3
        perpendicularBack = backLeft.encoder;//2

        parallelRight.setDirection(Motor.Direction.REVERSE);
        parallelLeft.setDirection(Motor.Direction.REVERSE);
        perpendicularBack.setDirection(Motor.Direction.REVERSE);

        resetEncoders();

//        /////////// add the localizer here once encoders are set up
//        localizer = new StandardTrackingWheelLocalizer(opmode.hardwareMap, );

    }

    public void strafeDistance(double distanceInch) {
        resetEncoders();

        long startTime = System.currentTimeMillis();
        long timeout = (long)(4000*Math.abs(distanceInch)/24.);

        double driveP = 0.0001;
//        double headingP = 0.3;


        int targetDist = inchesToTicks(distanceInch);

        int currentPos = getEncoderTicks()[2];
        int error = targetDist - currentPos;

        long lastTime = System.currentTimeMillis();
        int lastPos = currentPos;

        Telemetry tel = FtcDashboard.getInstance().getTelemetry();

        while(opmode.opModeIsActive() && (Math.abs(error) > 500 ||
                Math.abs((currentPos-lastPos)/(System.currentTimeMillis()-lastTime)) > 0.00001)
                && System.currentTimeMillis()-startTime < timeout) {

            lastTime = System.currentTimeMillis();
            lastPos = currentPos;

            currentPos = getEncoderTicks()[2];
            error = targetDist - currentPos;

            double drivePower = Range.clip(error*driveP, -.4, .4);

            double currentAngle = robot.imu.revIMU.getHeading();
            double turnError = currentAngle - targetHeading;

            double turnPower = Range.clip(turnError*headingP, -headingLimit, headingLimit);

            opmode.telemetry.addData("Mode", "STRAFE");
            opmode.telemetry.addData("TargetHeading", targetHeading);
            opmode.telemetry.addData("CurrrentHeading", currentAngle);
            opmode.telemetry.addData("TargetPos", targetDist);
            opmode.telemetry.addData("CurrentPos", currentPos);
            opmode.telemetry.update();

            drive(0, drivePower, turnPower, false);
        }
    }

    public void driveDistance(double distanceInch) {

        long startTime = System.currentTimeMillis();
        long timeout = (long)(4000*Math.abs(distanceInch)/24.);

        resetEncoders();

        double driveP = 0.0001;
//        double headingP = 0.3;


        int targetDist = inchesToTicks(distanceInch);

        int currentPos = getEncoderTicks()[1];
        int error = targetDist - currentPos;

        long lastTime = System.currentTimeMillis()-1;
        int lastPos = currentPos;

        Telemetry tel = FtcDashboard.getInstance().getTelemetry();



        while(opmode.opModeIsActive() && (Math.abs(error) > 500 ||
                Math.abs((currentPos-lastPos)/(System.currentTimeMillis()-lastTime)) > 0.00001)
                && System.currentTimeMillis()-startTime < timeout) {

            lastTime = System.currentTimeMillis();
            lastPos = currentPos;

            currentPos = getEncoderTicks()[1];
            error = targetDist - currentPos;

            double drivePower = Range.clip(error*driveP, -.4, .4);

            double currentAngle = robot.imu.revIMU.getHeading();
            double turnError = currentAngle - targetHeading;

            double turnPower = Range.clip(turnError*headingP, -headingLimit, headingLimit);

            opmode.telemetry.addData("Mode", "STRAIGHT");
            opmode.telemetry.addData("TargetHeading", targetHeading);
            opmode.telemetry.addData("CurrrentHeading", currentAngle);
            opmode.telemetry.addData("TargetPos", targetDist);
            opmode.telemetry.addData("CurrentPos", currentPos);
            opmode.telemetry.update();

            drive(drivePower, 0, turnPower, false);
        }

    }



    public void turnToHeading(double heading) {

        long startTime = System.currentTimeMillis();
        long timeout = 4000;

        targetHeading = -heading;

        double P = 0.03;

        double currentHeading = robot.imu.revIMU.getHeading();
        double error = currentHeading - targetHeading;

        double lastHeading = currentHeading;
        long lastTime = System.currentTimeMillis();

        while(opmode.opModeIsActive() && (Math.abs(error) > 1.5 ||
                Math.abs((currentHeading-lastHeading)/(System.currentTimeMillis()-lastTime)) > 0.0005)
                && System.currentTimeMillis()-startTime < timeout) {
            lastHeading = currentHeading;
            lastTime = System.currentTimeMillis();

            currentHeading = robot.imu.revIMU.getHeading();
            error = currentHeading - targetHeading;

            opmode.telemetry.addData("Mode", "TURN");
            opmode.telemetry.addData("Target", targetHeading);
            opmode.telemetry.addData("Currrent", currentHeading);
            opmode.telemetry.update();

            double power = Range.clip(error*P, -0.5, 0.5);
            drive(0, 0, power, false);

        }

    }



    public int inchesToTicks(double inches) {
        return (int)(8192*inches/(Math.PI*1.5));
    }

    public double ticksToInches(int ticks) {
        return (ticks/8192.)*(Math.PI*1.5);
    }

    public void drive(double forward, double strafe, double turn, boolean fieldCentric) {
        if(fieldCentric) {
            // maybe use odo for heading or average odo and imu? faster cycles maybe?
            drivebase.driveFieldCentric(strafe, forward, turn, robot.imu.revIMU.getHeading());
        } else {
            drivebase.driveRobotCentric(strafe*multiplier, forward*multiplier, turn*multiplier);
        }
    }

    public void resetEncoders() {
        parallelLeft.reset();
        parallelRight.reset();
        perpendicularBack.reset();
    }

    public int[] getEncoderTicks() {
        return new int[]{
                parallelLeft.getPosition(),
                parallelRight.getPosition(),
                perpendicularBack.getPosition()
        };
    }

}
