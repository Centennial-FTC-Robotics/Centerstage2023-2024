package org.centennialrobotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.centennialrobotics.subsystems.Camera;
import org.centennialrobotics.subsystems.Climber;
import org.centennialrobotics.subsystems.Drivetrain;
import org.centennialrobotics.subsystems.IMU;
import org.centennialrobotics.subsystems.Intake;
import org.centennialrobotics.subsystems.Outtake;

public class Robot {

    public Drivetrain drivetrain = new Drivetrain();
    public Outtake outtake = new Outtake();
    public Intake intake = new Intake();
    public IMU imu = new IMU();
    public Climber climber = new Climber();
    public Camera camera = new Camera();

    public Subsystem[] subsystems = new Subsystem[] {
        drivetrain, outtake, intake, imu, climber
    };

    public void initialize(LinearOpMode opmode) {

        for(Subsystem subsystem : subsystems) {
            subsystem.preInit(opmode, this);
        }


    }

}
