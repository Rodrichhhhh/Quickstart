package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.teamcode.mylib.RobotPos2;

@TeleOp(name = "Pellous_team")
public class Pellous_team extends LinearOpMode {

    private IMU imu;
    private RobotPos2 robotPos2;


    @Override
    public void runOpMode() {

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();



        // Gamepad tracking for toggles
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Motors
        DcMotor BottomLeftMotor;
        DcMotor TopRightMotor;
        DcMotor BottomRightMotor;
        DcMotor Collector;
        DcMotor AssistantShooter;
        DcMotor TopLMotor;



        // LEDs
        DigitalChannel led1;
        DigitalChannel led2;


        // Shooter motor (Ex)
        DcMotorEx ShooterMotor;

        //encoders
        DcMotor leftOdo, rightOdo, midOdo; //encoders




        // Initialize hardware
        TopLMotor = hardwareMap.get(DcMotor.class, "TopLeftMotor");
        BottomLeftMotor = hardwareMap.get(DcMotor.class, "BottomLeftMotor");
        TopRightMotor = hardwareMap.get(DcMotor.class, "TopRightMotor");
        BottomRightMotor = hardwareMap.get(DcMotor.class, "BottomRightMotor");
        Collector = hardwareMap.get(DcMotor.class, "Collector");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        AssistantShooter = hardwareMap.get(DcMotor.class, "AssistantShooter");
        led1 = hardwareMap.digitalChannel.get("redled");
        led2 = hardwareMap.digitalChannel.get("blueled");


        leftOdo = hardwareMap.get(DcMotor.class, "BottomLeftMotor"); // temp, ideally a real encoder
        rightOdo = hardwareMap.get(DcMotor.class, "TopRightMotor");
        midOdo = hardwareMap.get(DcMotor.class, "midOdo"); //midOdo


        led1.setMode(DigitalChannel.Mode.OUTPUT);
        led2.setMode(DigitalChannel.Mode.OUTPUT);

        AssistantShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        midOdo.setDirection(DcMotorSimple.Direction.REVERSE);


        // Motor directions
        TopLMotor.setDirection(DcMotor.Direction.REVERSE);
        Collector.setDirection(DcMotor.Direction.REVERSE);
        ShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        TopLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TopRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BottomLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BottomRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // ===== IMU SETUP =====
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD         //might change
                )
        );

        imu.initialize(parameters);

        robotPos2 = new RobotPos2(
                TopLMotor,
                BottomLeftMotor,
                TopRightMotor,
                BottomRightMotor,

                BottomLeftMotor,   // left odo
                TopRightMotor,     // right odo
                midOdo,            // strafe odo (NEW)

                imu
        );


        waitForStart();

        double MAX_POWER = -1.0;
        double HALF_POWER = -0.5;
        double QUARTER_POWER = -0.25;

        double NO_POWER = 0.0;
        double PER_REV = 28;


        boolean MAXSHOOT = false;
        boolean MINSHOOT = false;


        double rpmLimit1 = 2700;
        double limit1 = 3200;
        double rpmLimit2 = 3700;
        double limit2 = 3950;


        boolean LONGON = false;
        boolean SHORTON = false;


//        RobotPose robotPose = new RobotPose(BottomLeftMotor, BottomRightMotor, TopLMotor, TopRightMotor, leftOdo, rightOdo, midOdo); //Rodo's crazy library



        //double rampedPower = 0.0;
        //double rampRate = 0.004;  // Power added each loop

        while (opModeIsActive()) {


            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // Intake toggle for shooter
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                MAXSHOOT = !MAXSHOOT;
            }

            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                MINSHOOT = !MINSHOOT;
                // make sure high is off when low is toggled
            }


            // shooter

            double voltage = voltageSensor.getVoltage();
            double nominalVoltage = 13.0;  // voltage you tuned kF at
            double voltageComp = nominalVoltage / voltage;

            //sigmoid math
            double targeted = 3750.0;
            double targeted2 = 2700.0;

            double kF2 = 0.67;
            double kF = 0.80;
            double k = 0.002;
            double kP = 0.15;

            double currentRPM = (ShooterMotor.getVelocity() / PER_REV) * 60;

            double error = targeted - currentRPM;
            double sigmoid = (2.0 / (1.0 + Math.exp(-k * error))) - 1.0;
            double correction = sigmoid * kP;

            double POWER = (kF * voltageComp) + correction;
            POWER = Math.max(0.0, Math.min(1.0, POWER));

            double error2 = targeted2 - currentRPM;
            double sigmoid2 = (2.0 / (1.0 + Math.exp(-k * error2))) - 1.0;
            double correction2 = sigmoid2 * kP;

            double POWER2 = (kF2 * voltageComp) + correction2;
            POWER2 = Math.max(0.0, Math.min(1.0, POWER2));


            boolean LONG = false;
            boolean SHORT = false;


            double targetPower;
            if (MAXSHOOT) {
                double minPower = POWER - 0.05;
                targetPower = Range.clip(POWER, minPower, 1.0);
                LONGON = true;
            } else if (MINSHOOT) {
                double minPower = POWER2 - 0.05;
                targetPower = Range.clip(POWER2, minPower, 1.0);
                SHORTON = true;

            } else {
                targetPower = 0.0;  // Shooter completely off
                SHORTON = false;
                LONGON = false;
            }

// Apply power
            ShooterMotor.setPower(targetPower);

            led1.setState(true);
            led2.setState(true);

            // LED logic based on RPM + long + short
            if (currentRPM >= rpmLimit1 && currentRPM <= limit1 && SHORTON) {
                SHORT = true;
                led1.setState(false);
            } else if (currentRPM >= rpmLimit2 && currentRPM < limit2 && LONGON) {
                LONG = true;
                led1.setState(false);
                led2.setState(false);
            }

            //power to assistant shooter


            //power to assistant shooter
            if (LONG && gamepad2.a) {
                AssistantShooter.setPower(-0.7);        //adjust speed
                Collector.setPower(-0.6);               //adjust speed
            } else if (SHORT && gamepad2.a) {
                AssistantShooter.setPower(-0.7);        //adjust speed
                Collector.setPower(-0.7);               //adjust speed
            } else if (gamepad2.y) {
                Collector.setPower(MAX_POWER);
            } else if (gamepad2.b) {
                AssistantShooter.setPower(HALF_POWER);

            } else if (gamepad2.x) {
                Collector.setPower(-MAX_POWER);
            }
            else {
                AssistantShooter.setPower(NO_POWER);
                Collector.setPower(NO_POWER);
            }


//            telemetry.addData("Power", targetPower);
//            telemetry.addData("maxshot", MAXSHOOT);
//            telemetry.addData("minshot", MINSHOOT);
//            telemetry.addData("RPM", currentRPM);
//            telemetry.addData("Short:", SHORT);
//            telemetry.addData("Long:", LONG);
//            telemetry.addData("Short_on", SHORTON);
//            telemetry.addData("Long_on", LONGON);
//            telemetry.update();


            // Drive control
            double rotate = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;   // flip left/right
            double forward = -gamepad1.left_stick_y;

            TopLMotor.setPower(forward + rotate + strafe);
            BottomLeftMotor.setPower(forward + rotate - strafe);
            TopRightMotor.setPower(forward - rotate - strafe);
            BottomRightMotor.setPower(forward - rotate + strafe);

            robotPos2.update(gamepad1.a, gamepad1.b, gamepad1.y);

            if (gamepad1.x) {
                robotPos2.recordShot();
            }

//            if (gamepad1.right_bumper) {
//                robotPos2.clearMemory();
//            }



            telemetry.addData("X", robotPos2.getX());
            telemetry.addData("Y", robotPos2.getY());
            telemetry.addData("Theta (deg)", Math.toDegrees(robotPos2.getTheta()));
            telemetry.addData("Theta", robotPos2.getTheta());
            telemetry.addData("Memory Active", robotPos2.isMemoryActive() ? "ON" : "OFF");
            telemetry.addData("RPM", currentRPM);
            telemetry.update();


        }


    }

}


