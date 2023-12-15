package org.centennialrobotics.subsystems;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.centennialrobotics.Subsystem;

public class IMU extends Subsystem {

    public RevIMU revIMU;

    public void init(LinearOpMode opmode) {

        revIMU = new RevIMU(opmode.hardwareMap);
        revIMU.init();
    }

}
