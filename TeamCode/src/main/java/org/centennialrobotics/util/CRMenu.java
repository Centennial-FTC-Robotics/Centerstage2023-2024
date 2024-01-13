package org.centennialrobotics.util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CRMenu<T> {

    T option1;
    T option2;
    T option3;
    T option4;

    public CRMenu(T o1, T o2, T o3, T o4) {
        this.option1 = o1;
        this.option2 = o2;
        this.option3 = o3;
        this.option4 = o4;
    }

    public T get(LinearOpMode opmode) {
        boolean selected = false;
        GamepadEx[] gamepads = new GamepadEx[]
                {new GamepadEx(opmode.gamepad1), new GamepadEx(opmode.gamepad2)};

        while(!selected && (!opmode.isStopRequested())) {
            if(option1 != null)
                opmode.telemetry.addData("UP >>", option1.toString());
            if(option2 != null)
                opmode.telemetry.addData("DOWN >>", option2.toString());
            if(option3 != null)
                opmode.telemetry.addData("LEFT >>", option3.toString());
            if(option4 != null)
                opmode.telemetry.addData("RIGHT >>", option4.toString());
            opmode.telemetry.update();

            for(GamepadEx gpad : gamepads) {
                if(gpad.wasJustPressed(GamepadKeys.Button.DPAD_UP) && option1 != null)
                    return option1;
                if(gpad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) && option2 != null)
                    return option2;
                if(gpad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) && option3 != null)
                    return option3;
                if(gpad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) && option4 != null)
                    return option4;

                gpad.readButtons();
            }

        }
        return null;
    }



}
