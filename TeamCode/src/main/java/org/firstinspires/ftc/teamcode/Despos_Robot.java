package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name = "Despos_Robot", group = "Pedro")
public class Despos_Robot extends LinearOpMode {
    @Override
    public void runOpMode() {

        // Gamepad tracking for toggles
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Motors

        DcMotor BottomLeftMotor;
        DcMotor TopRightMotor;
        DcMotor BottomRightMotor;
        DcMotor Collector;
        DcMotor AssistantShooter;
        DcMotor TopLeftMotor;



// LEDs
        DigitalChannel led1;
        DigitalChannel led2;


// Shooter motor (Ex)
        DcMotorEx ShooterMotor;


        // Initialize hardware
        TopLeftMotor = hardwareMap.get(DcMotor.class, "TopLeftMotor");
        BottomLeftMotor = hardwareMap.get(DcMotor.class, "BottomLeftMotor");
        TopRightMotor = hardwareMap.get(DcMotor.class, "TopRightMotor");
        BottomRightMotor = hardwareMap.get(DcMotor.class, "BottomRightMotor");
        Collector = hardwareMap.get(DcMotor.class, "Collector");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        AssistantShooter = hardwareMap.get(DcMotor.class, "AssistantShooter");
        led1 = hardwareMap.digitalChannel.get("redled");
        led2 = hardwareMap.digitalChannel.get("blueled");


        led1.setMode(DigitalChannel.Mode.OUTPUT);
        led2.setMode(DigitalChannel.Mode.OUTPUT);


        // Motor directions
        AssistantShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Collector.setDirection(DcMotorSimple.Direction.REVERSE);
        BottomRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        TopLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        double MAX_POWER = -1.0;
        double HALF_POWER = -0.5;
        double QUARTER_POWER = -0.25;

        double NO_POWER = 0.0;
        double PER_REV = 28;


        boolean MAXSHOOT = false;
        boolean MINSHOOT = false;


        double rpmLimit1 = 3000;        //MIN RPM close shot
        double limit1 = 3400;           //MAX RPM close shot
        double rpmLimit2 = 4250;        //MIN RPM far shot
        double limit2 = 4500;           //MAX RPM far shot


        boolean LONGON = false;
        boolean SHORTON = false;




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
            //sigmoid math
            double targeted = 4500;         //far shot target RPM
            double targeted2 = 3250;        //close shot target RPM

            double kF = 0.90;       //avg shooting power for far shot
            double k = 0.002;
            double kP = 0.15;

            double currentRPM = (ShooterMotor.getVelocity() / PER_REV) * 60;

            double error = targeted - currentRPM;
            double sigmoid = (2.0 / (1.0 + Math.exp(-k * error))) - 1.0;
            double correction = sigmoid * kP;
            double POWER = kF + correction;
            POWER = Math.max(0.0, Math.min(1.0, POWER));


            double kF2 = 0.75;      //avg shor power for close shot

            double error2 = targeted2 - currentRPM;
            double sigmoid2 = (2.0 / (1.0 + Math.exp(-k * error2))) - 1.0;
            double correction2 = sigmoid2 * kP;
            double POWER2 = kF2 + correction2;
            POWER2 = Math.max(0.0, Math.min(1.0, POWER2));


            double CURRENT_RPM = (ShooterMotor.getVelocity() / PER_REV) * 60;

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
            if (CURRENT_RPM >= rpmLimit1 && CURRENT_RPM <= limit1 && SHORTON) {
                SHORT = true;
                led1.setState(false);
            } else if (CURRENT_RPM >= rpmLimit2 && CURRENT_RPM < limit2 && LONGON) {
                LONG = true;
                led1.setState(false);
                led2.setState(false);
            }

            //power to assistant shooter
            double BallShot = -0.75;



            //power to assistant shooter
            if (LONG && gamepad2.a) {
                AssistantShooter.setPower(-MAX_POWER);
                Collector.setPower(-0.25);
            } else if (SHORT && gamepad2.a) {
                AssistantShooter.setPower(-MAX_POWER);
                Collector.setPower(BallShot);
            } else if (gamepad2.y) {
                Collector.setPower(MAX_POWER);
            } else if (gamepad2.b) {
                AssistantShooter.setPower(-MAX_POWER);

            } else {
                AssistantShooter.setPower(NO_POWER);
                Collector.setPower(NO_POWER);
            }



            telemetry.addData("Power", targetPower);
            telemetry.addData("maxshot", MAXSHOOT);
            telemetry.addData("minshot", MINSHOOT);
            telemetry.addData("RPM", CURRENT_RPM);
            telemetry.addData("Short:", SHORT);
            telemetry.addData("Long:", LONG);
            telemetry.addData("Short_on", SHORTON);
            telemetry.addData("Long_on", LONGON);
            telemetry.update();







            // Drive control
            double rotate = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;   // flip left/right
            double forward = -gamepad1.left_stick_y;


            TopLeftMotor.setPower(forward + rotate + strafe);
            BottomLeftMotor.setPower(forward + rotate - strafe);
            TopRightMotor.setPower(forward - rotate - strafe);
            BottomRightMotor.setPower(forward - rotate + strafe);


        }

    }
}
