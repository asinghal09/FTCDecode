package org.firstinspires.ftc.teamcode.MeetCode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Transfer {
    private DcMotor transfer;
    public static double transferSpeed = 1;

    public Transfer(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotor.class, "transfer");
    }

    public Action transferOn() {
        return p -> {
            transfer.setPower(transferSpeed);
            return false;
        };
    }
    public Action transferOnSlow() {
        return p -> {
            transfer.setPower(0.35);
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
