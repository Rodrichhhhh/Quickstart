package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name = "coffee", group = "Pedro")
public class coffee extends LinearOpMode {
    @Override
    public void runOpMode() {


        DcMotor convey;

        convey = hardwareMap.get(DcMotor.class, "convey");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                convey.setPower(1);
            }else {
                convey.setPower(0);
            }

    }
    }
}
