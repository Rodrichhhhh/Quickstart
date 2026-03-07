package org.firstinspires.ftc.teamcode.Autonomous;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Autotos Red Side 12 Balls")
public class AutotosREDSIDE_12 extends OpMode {

    private Follower follower;

    private int pathState = 0;

    private Timer shooterTimer = new Timer();
    private Timer collectorTimer = new Timer();
    private Timer stallStartTime = new Timer();

    private Timer EndTimer = new Timer();


    boolean stallDetected = false;

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

    // START POSE MUST MATCH PATH1 START
    private final Pose startPose =
            new Pose(123, 123.000, Math.toRadians(36));

    double lastShooterRPM = 0;
    int shooterCount = 0;

    static final double PER_REV = 28;
    static final double S_MAX_RPM_DROP = 200;

    DcMotor AssistantShooter;
    DcMotorEx ShooterMotor, Collector;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        Collector = hardwareMap.get(DcMotorEx.class, "Collector");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        AssistantShooter = hardwareMap.get(DcMotor.class, "AssistantShooter");

        telemetry.addLine("Initialized");
        telemetry.update();
    }


    private void buildPaths() {

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.000, 123.000),

                                new Pose(102.729, 102.280)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(40))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(102.729, 102.280),
                                new Pose(92.280, 88.897),
                                new Pose(131.402, 92.953)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(131.402, 92.953),

                                new Pose(99.365, 98.243)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.365, 98.243),
                                new Pose(85.963, 75.804),
                                new Pose(128.747, 76.935)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.747, 76.935),

                                new Pose(98.467, 94.206)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(98.467, 94.206),
                                new Pose(79.332, 44.351),
                                new Pose(127.486, 45.916)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.747, 76.935),

                                new Pose(98.467, 94.206)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))

                .build();

    }



    @Override
    public void start() {

        shooterTimer.resetTimer();
        collectorTimer.resetTimer();

        follower.followPath(Path1);
        pathState = 1;
    }

    @Override
    public void loop() {

        follower.update();

        double shooterRPM = Math.abs(ShooterMotor.getVelocity()) / PER_REV * 60;
        double collectorRPM = Math.abs(Collector.getVelocity()) / PER_REV * 60;

        switch (pathState) {

            // ================= DRIVE TO FIRST SHOOT POSITION =================
            case 1:

                if (!follower.isBusy()) {

                    shooterTimer.resetTimer();
                    EndTimer.resetTimer();
                    pathState = 2;
                }

                break;


            // ================= SHOOT FIRST 3 =================
            case 2:

                shootLogic();

                if (shooterTimer.getElapsedTime() >= 150) {

                    if (lastShooterRPM - shooterRPM > S_MAX_RPM_DROP) {
                        shooterCount++;
                    }

                    lastShooterRPM = shooterRPM;
                    shooterTimer.resetTimer();
                }

                if (shooterCount >= 3 || EndTimer.getElapsedTime() >= 5000) {

                    shooterCount = 0;

                    stopShooter();

                    collectorTimer.resetTimer();

                    follower.followPath(Path2);
                    pathState = 3;
                }

                break;


            // ================= DRIVE PATH2 + COLLECT =================
            case 3:

                // collector runs WHILE driving
                Collector.setPower(0.7);

                if (!follower.isBusy()) {

                    if (collectorRPM <= 3 || collectorTimer.getElapsedTime() >= 3000) {

                        if (!stallDetected) {
                            stallStartTime.resetTimer();
                            stallDetected = true;
                        }

                        if (stallStartTime.getElapsedTime() >= 250) {

                            Collector.setPower(0);
                            stallDetected = false;

                            follower.followPath(Path3);
                            pathState = 4;
                        }

                    } else {
                        stallDetected = false;
                    }
                }

                break;


            // ================= DRIVE BACK TO SHOOT =================
            case 4:

                if (!follower.isBusy()) {

                    shooterTimer.resetTimer();
                    EndTimer.resetTimer();

                    pathState = 5;
                }

                break;


            // ================= SHOOT SECOND SET =================
            case 5:

                shootLogic();

                if (shooterTimer.getElapsedTime() >= 150) {

                    if (lastShooterRPM - shooterRPM > S_MAX_RPM_DROP) {
                        shooterCount++;
                    }

                    lastShooterRPM = shooterRPM;
                    shooterTimer.resetTimer();
                }

                if (shooterCount >= 4 || EndTimer.getElapsedTime() >= 5000) {

                    shooterCount = 0;
                    stopShooter();

                    follower.followPath(Path4);
                    pathState = 6;
                }

                break;


            case 6:

                // collector runs WHILE driving
                Collector.setPower(0.7);

                if (!follower.isBusy()) {

                    if (collectorRPM <= 3 || collectorTimer.getElapsedTime() >= 3000) {

                        if (!stallDetected) {
                            stallStartTime.resetTimer();
                            stallDetected = true;
                        }

                        if (stallStartTime.getElapsedTime() >= 250) {

                            Collector.setPower(0);
                            stallDetected = false;

                            follower.followPath(Path5);
                            pathState = 7;
                        }

                    } else {
                        stallDetected = false;
                    }
                }

                break;



            case 7:


                if (!follower.isBusy()) {

                    shooterTimer.resetTimer();
                    EndTimer.resetTimer();

                    pathState = 5;
                }

                break;


            case 8:


                shootLogic();

                if (shooterTimer.getElapsedTime() >= 150) {

                    if (lastShooterRPM - shooterRPM > S_MAX_RPM_DROP) {
                        shooterCount++;
                    }

                    lastShooterRPM = shooterRPM;
                    shooterTimer.resetTimer();
                }

                if (shooterCount >= 4|| EndTimer.getElapsedTime() >= 8000) {

                    shooterCount = 0;
                    stopShooter();

                    follower.followPath(Path6);


                    pathState = 9;

                }



            case 9:

                // collector runs WHILE driving
                Collector.setPower(0.7);

                if (!follower.isBusy()) {

                    if (collectorRPM <= 3 || collectorTimer.getElapsedTime() >= 3000) {

                        if (!stallDetected) {
                            stallStartTime.resetTimer();
                            stallDetected = true;
                        }

                        if (stallStartTime.getElapsedTime() >= 250) {

                            Collector.setPower(0);
                            stallDetected = false;

                            follower.followPath(Path7);
                            pathState = 10;
                        }

                    } else {
                        stallDetected = false;
                    }
                }

                break;



            case 10:
                if (!follower.isBusy()) {

                    shooterTimer.resetTimer();
                    EndTimer.resetTimer();

                    pathState = 11;
                }

                break;


            case 11:
                shootLogic();

                if (shooterTimer.getElapsedTime() >= 150) {

                    if (lastShooterRPM - shooterRPM > S_MAX_RPM_DROP) {
                        shooterCount++;
                    }

                    lastShooterRPM = shooterRPM;
                    shooterTimer.resetTimer();
                }

                if (shooterCount >= 4) {

                    shooterCount = 0;
                    stopShooter();

                    pathState = 12;

                }




        }

        telemetry.addData("State", pathState);
        telemetry.addData("Shooter RPM", shooterRPM);
        telemetry.addData("Collector RPM", collectorRPM);
        telemetry.addData("Shots", shooterCount);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();

    }

    // SHOOTER CONTROL
    public void shootLogic () {

        double READY_LOW = 2650;
        double READY_HIGH = 3000;

        double targeted2 = 2700.0;

        double kF2 = 0.52;
        double k = 0.002;
        double kP = 0.15;

        double Now_RPM = (ShooterMotor.getVelocity() / PER_REV) * 60;

        double error2 = targeted2 - Now_RPM;
        double sigmoid2 = (2.0 / (1.0 + Math.exp(-k * error2))) - 1.0;
        double correction2 = sigmoid2 * kP;

        double POWER2 = kF2 + correction2;
        POWER2 = Math.max(0.0, Math.min(1.0, POWER2));

        double minPower = POWER2 - 0.05;
        double targetPower = Range.clip(POWER2, minPower, 1.0);

        ShooterMotor.setPower(-targetPower);
        double Pos_RPM = -Now_RPM;

        if (Pos_RPM >= READY_LOW && Pos_RPM <= READY_HIGH) {
            Collector.setPower(0.7);
            AssistantShooter.setPower(1);
        } else {
            Collector.setPower(0);
            AssistantShooter.setPower(0);
        }
        telemetry.addData("Now RPM", Now_RPM);
    }

    public void stopShooter () {

        ShooterMotor.setPower(0);
        AssistantShooter.setPower(0);
        Collector.setPower(0);
    }
}
