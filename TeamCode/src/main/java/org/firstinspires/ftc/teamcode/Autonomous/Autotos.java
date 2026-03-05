

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

@Autonomous(name = "Pedro Testing Auto FIXED")
public class Autotos extends OpMode {

    private Follower follower;

    private int pathState = 0;

    private Timer shooterTimer = new Timer();
    private Timer collectorTimer = new Timer();
    private Timer stallStartTime = new Timer();

    boolean stallDetected = false;
    boolean collectorStarted = false;

    private PathChain Path1, Path2, Path3;

    private final Pose startPose =
            new Pose(123.000, 123.000, Math.toRadians(36));

    // Shooter variables
    double lastShooterRPM = 0;
    int shooterCount = 0;
    double PER_REV = 28;
    static final double S_MAX_RPM_DROP = 200;


    private int lastState = -1;



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

                                    new Pose(98.916, 103.177)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(40))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(98.916, 103.177),
                                    new Pose(92.280, 88.897),
                                    new Pose(131.402, 92.953)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.402, 92.953),

                                    new Pose(91.290, 91.290)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))

                    .build();
        }

    @Override
    public void start() {
        follower.followPath(Path1);
        pathState = 1;
    }

    @Override
    public void loop() {

        if (pathState != lastState) {
            collectorTimer.resetTimer();
            stallStartTime.resetTimer();
            lastState = pathState;
        }

        double shooterRPM = Math.abs(ShooterMotor.getVelocity()) / PER_REV * 60;
        double collectorRPM = Math.abs(Collector.getVelocity()) / PER_REV * 60;

        follower.update();

        switch (pathState) {




            // ================= SHOOT FIRST SET =================
            case 1:

                if (!follower.isBusy()) {

                    shootLogic(shooterRPM);

                    if (shooterTimer.getElapsedTime() == 0) {
                        shooterTimer.resetTimer();
                    }

                    if (shooterTimer.getElapsedTime() >= 80) {

                        if (lastShooterRPM - shooterRPM > S_MAX_RPM_DROP) {
                            shooterCount++;
                        }

                        lastShooterRPM = shooterRPM;

                        // Reset timer for next check
                        shooterTimer.resetTimer();
                    }

                    // After 3 shots, stop and continue
                    if (shooterCount >= 3) {

                        shooterCount = 0;

                        stopShooter();

                        follower.followPath(Path3);
                        pathState = 2;
                    }
                }
                break;




            // ================= COLLECT =================
            case 2:

                Collector.setPower(0.7);

                if (collectorRPM <= 3 || collectorTimer.getElapsedTime() >= 5000) {

                    if (!stallDetected) {
                        stallStartTime.resetTimer();
                        stallDetected = true;
                    }

                    if (stallStartTime.getElapsedTime() >= 250) {

                        Collector.setPower(0);

                        stallDetected = false;

                        follower.followPath(Path2);
                        pathState = 3;
                    }

                } else {
                    stallDetected = false;
                }

                break;



            case 3:

                if (!follower.isBusy()) {

                    follower.followPath(Path3);
                    pathState = 4;

                }

                break;

            case 4:

                if (!follower.isBusy()) {

                    shootLogic(shooterRPM);

                    if (shooterTimer.getElapsedTime() == 0) {
                        shooterTimer.resetTimer();
                    }

                    if (shooterTimer.getElapsedTime() >= 80) {

                        if (lastShooterRPM - shooterRPM > S_MAX_RPM_DROP) {
                            shooterCount++;
                        }

                        lastShooterRPM = shooterRPM;

                        // Reset timer for next check
                        shooterTimer.resetTimer();
                    }

                    // After 3 shots, stop and continue
                    if (shooterCount >= 3) {

                        shooterCount = 0;

                        stopShooter();

                        follower.followPath(Path3);
                        pathState = 5;
                    }
                }
                break;

        }

        telemetry.addData("State", pathState);
        telemetry.addData("Shooter RPM", shooterRPM);
        telemetry.addData("Collector RPM", collectorRPM);
        telemetry.addData("Shots Fired", shooterCount);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    // ================= SHOOTING LOGIC =================
    public void shootLogic(double currentRPM) {

        double READY_LOW = 2750;
        double READY_HIGH = 3100;

        double targeted2 = 2800.0;

        double kF2 = 0.55;
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

    public void Collecting() {
        Collector.setPower(0.7);
    }

    public void NoCollecting() {
        Collector.setPower(0);
    }

    public void stopShooter() {
        ShooterMotor.setPower(0);
        AssistantShooter.setPower(0);
        Collector.setPower(0);
    }
}