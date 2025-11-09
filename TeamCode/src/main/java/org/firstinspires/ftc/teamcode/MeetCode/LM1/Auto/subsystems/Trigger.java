package org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config

public class Trigger {
    private Servo trigger;
    public static double downPos = 0.38;
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
