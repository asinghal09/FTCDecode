package org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Flywheels {
    private DcMotorEx leftFlywheel, rightFlywheel;
    public static double fWheelPower = 0.9;
    public Flywheels(HardwareMap hardwareMap) {
        leftFlywheel  = hardwareMap.get(DcMotorEx.class, "wheelLeft");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "wheelRight");

        // Reverse one side so they spin in the same physical direction
        leftFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /*
    public void setPower(double p) {
        power = p;
        leftFlywheel.setPower(p);
        rightFlywheel.setPower(p);
    }

     */

    public Action wheelsOn() {
        return t -> {
            leftFlywheel.setPower(fWheelPower);
            rightFlywheel.setPower(fWheelPower);
            return false;   // run once
        };
    }
    public Action wheelsOff() {
        return t -> {
            leftFlywheel.setPower(0);
            rightFlywheel.setPower(0);
            return false;   // run once
        };
    }
}
