package org.centennialrobotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.centennialrobotics.subsystems.Drivetrain;
import org.centennialrobotics.subsystems.IMU;
import org.centennialrobotics.subsystems.Intake;
import org.centennialrobotics.subsystems.Outtake;

public class Robot {

    public Drivetrain drivetrain = new Drivetrain();
    public Outtake outtake = new Outtake();
    public Intake intake = new Intake();
    public IMU imu = new IMU();

    public Subsystem[] subsystems = new Subsystem[] {
        drivetrain, outtake, intake, imu
    };

    public void initialize(LinearOpMode opmode) {

        for(Subsystem subsystem : subsystems) {
            subsystem.preInit(opmode, this);
        }

    }

}
