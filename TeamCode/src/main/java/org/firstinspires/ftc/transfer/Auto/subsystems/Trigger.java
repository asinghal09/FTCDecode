package org.firstinspires.ftc.transfer.Auto.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Trigger {
    private Servo trigger;
    public static double downPos = 0.3;
    public static double upPos = 1;
    public Trigger(HardwareMap hardwareMap) {
        trigger = hardwareMap.get(Servo.class, "trigger");
    }

    public Action triggerDown() {
        return p -> {
            trigger.setPosition(downPos);
            return false;
        };
    }

    public Action triggerUp() {
        return p -> {
            trigger.setPosition(upPos);
            return false;
        };
    }
}
