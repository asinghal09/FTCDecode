package org.firstinspires.ftc.transfer.Auto.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {
    private CRServo transfer;
    public static double transferSpeed = 1;

    public Transfer(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(CRServo.class, "transfer");
    }

    public Action transferOn() {
        return p -> {
            transfer.setPower(transferSpeed);
            return false;
        };
    }

    public Action transferOff() {
        return p -> {
            transfer.setPower(0.0);
            return false;
        };
    }
}
