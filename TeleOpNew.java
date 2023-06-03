package org.firstinspires.ftc.teamcode.edgemont.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.edgemont.auto.drive.Drive;
import org.firstinspires.ftc.teamcode.edgemont.lib.Grabber;
import org.firstinspires.ftc.teamcode.edgemont.lib.Grabber2;
import org.firstinspires.ftc.teamcode.edgemont.lib.Slide;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="New TeleOp")

public class TeleOpNew {
    DcMotor wheelLF;
    DcMotor wheelRF;
    DcMotor wheelRB;
    DcMotor wheelLB;
    BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;

    public void runOpMode() throws InterruptedException{

        wheelLF = hardwareMap.dcMotor.get("wheelLF");
        wheelRF = hardwareMap.dcMotor.get("wheelRF");
        wheelLB = hardwareMap.dcMotor.get("wheelLB");
        wheelRB = hardwareMap.dcMotor.get("wheelRB");


        double v1 = 0, v2 = 0, v3 = 0, v4 = 0;

        double forback = gamepad1.left_stick_y;
        double rightleft = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

            v1 += -forback + rightleft + strafe;
            v2 += -forback - rightleft - strafe;
            v3 += forback - rightleft + strafe;
            v4 += forback + rightleft - strafe;

        wheelLF.setPower(v1);
        wheelRF.setPower(v2);
        wheelLB.setPower(v3);
        wheelRB.setPower(v4);

        telemetry.addData("LeftFrontPower", v1);
        telemetry.addData("RightFrontPower", v2);
        telemetry.addData("LeftBackPower", v3);
        telemetry.addData("RightBackPower", v4);
        telemetry.update();
    }
}
