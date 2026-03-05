package org.firstinspires.ftc.teamcode.Autonomous;

// to do list:
// tune Max Collector + Shooter RPM drop
// tune the wait time for the Last rpm / current rpm for Collector + Shooter

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;






@Autonomous(name = "Pathing2")
public class Autotos1 extends LinearOpMode {

    DcMotor TopRightMotor, BottomLeftMotor, BottomRightMotor, AssistantShooter, TopLeftMotor;
    DcMotor leftOdo, rightOdo, mid0do; //encoders
    DcMotorEx ShooterMotor, Collector;    //Ex motor for RPM and velocity measurements


    //crazy math for the encoders


    //Shooter
    double PER_REV = 28;


    //Shooter stopping detector
    double LastShooterRPM = 0;
    int ShooterCount = 0;
    static final double S_MAX_RPM_DROP = 200;

    ElapsedTime shooterTime = new ElapsedTime();
    ElapsedTime pidTimer = new ElapsedTime();


    //Collector
    int CollectCount = 0;
    double LastCollectorRPM = 0;
    static final double C_MAX_RPM_DROP = 400;
    boolean CollectWork = true;




    enum AutoState {
        SHORT_SHOT,
        COLLECTOR

    }

    AutoState state = AutoState.SHORT_SHOT;


    @Override
    public void runOpMode() {

        TopLeftMotor = hardwareMap.get(DcMotor.class, "TopLeftMotor");
        BottomLeftMotor = hardwareMap.get(DcMotor.class, "BottomLeftMotor");
        TopRightMotor = hardwareMap.get(DcMotor.class, "TopRightMotor");
        BottomRightMotor = hardwareMap.get(DcMotor.class, "BottomRightMotor");
        Collector = hardwareMap.get(DcMotorEx.class, "Collector");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        AssistantShooter = hardwareMap.get(DcMotor.class, "AssistantShooter");

        leftOdo = hardwareMap.get(DcMotor.class, "BottomLeftMotor"); // temp, ideally a real encoder
        rightOdo = hardwareMap.get(DcMotor.class, "TopRightMotor");
        mid0do = hardwareMap.get(DcMotor.class, "midOdo"); //mid0do

        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        AssistantShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        mid0do.setDirection(DcMotorSimple.Direction.REVERSE);
        BottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BottomRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        AssistantShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mid0do.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BottomLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TopLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TopRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        waitForStart();



        while (opModeIsActive()) {
            pidTimer.reset();
            shooterTime.reset();


            double RPM = (ShooterMotor.getVelocity() / PER_REV) * 60;

            switch (state) {



                case SHORT_SHOT:
                    ShortShot();

                    if (shooterTime.seconds() == 0) shooterTime.reset();

                    shooterTime.reset();

                    if (LastShooterRPM - RPM > S_MAX_RPM_DROP) {
                        ShooterCount++;
                    }

                    sleep(80);
                    LastShooterRPM = RPM;

                    if (ShooterCount == 3) {
                        ShooterCount = 0;
                        state = AutoState.COLLECTOR;
                    }

                    telemetry.addData("Balls shot", ShooterCount);
                    telemetry.addData("seconds", shooterTime.seconds());
                    telemetry.update();
                    break;




                case COLLECTOR:
                    Collector();
                    if (!CollectWork) {
                        CollectWork = true;
                        state = AutoState.COLLECTOR;
                    }
                    break;

            }


        }
    }




    public void ShortShot() {

        double MIN_RPM = 3250.0;
        double READY_RPM_LOW = 2880;
        double READY_RPM_HIGH = 3500;

        double targetVelocity = (MIN_RPM / 60.0) * PER_REV;
        ShooterMotor.setVelocity(-targetVelocity);

        double shooterRPM = Math.abs(ShooterMotor.getVelocity()) / PER_REV * 60;

        if (shooterRPM >= READY_RPM_LOW && shooterRPM <= READY_RPM_HIGH) {
            Collector.setPower(0.7);
            AssistantShooter.setPower(1);
        } else {
            Collector.setPower(0);
            AssistantShooter.setPower(0);
        }

        telemetry.addData("Limited To", MIN_RPM);
        telemetry.addData("Current RPM", shooterRPM);
        telemetry.update();
    }



    public void Collector(){

        Collector.setPower(0.8);



        double CollectorRPM = (Collector.getVelocity() / PER_REV) * 60;

        if (LastCollectorRPM - CollectorRPM > C_MAX_RPM_DROP){
            CollectCount++;
        }

        LastCollectorRPM = CollectorRPM;

        if (CollectCount == 2) {
            AssistantShooter.setPower(0.5);
            CollectCount = 0;
            sleep(50);

        }

        double rpmChange = Math.abs(CollectorRPM - LastCollectorRPM);

        if (CollectorRPM < 50 && rpmChange < 30) {
            Collector.setPower(0);
            CollectWork = false;
        }

        LastCollectorRPM = CollectorRPM;



    }



}






