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

@Autonomous(name = "Autotos LONG Blue  FIXED")
public class AutotosLONGBLUE extends OpMode {

    private Follower follower;

    private int pathState = 0;

    private Timer shooterTimer = new Timer();
    private Timer collectorTimer = new Timer();
    private Timer stallStartTime = new Timer();

    boolean stallDetected = false;

    private PathChain Path1, Path2, Path3;

    // START POSE MUST MATCH PATH1 START
    private final Pose startPose =
            new Pose(56, 8, Math.toRadians(90));

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
                                new Pose(56.000, 8.000),

                                new Pose(56.000, 10.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 10.000),
                                new Pose(62.818, 39.626),
                                new Pose(91.215, 27.51401869158878)
                        )
                ).setTangentHeadingInterpolation()

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



            case 1:
                if (!follower.isBusy()) {

                    shooterTimer.resetTimer();
                    pathState = 2;
                }

                break;



            case 2:

                shootLogic();

                if (shooterTimer.getElapsedTime() >= 80) {

                    if (lastShooterRPM - shooterRPM > S_MAX_RPM_DROP) {
                        shooterCount++;
                    }

                    lastShooterRPM = shooterRPM;
                    shooterTimer.resetTimer();
                }

                if (shooterCount >= 3) {

                    shooterCount = 0;

                    stopShooter();

                    collectorTimer.resetTimer();

                    follower.followPath(Path2);
                    pathState = 3;
                }

                break;



            case 3:

                if (!follower.isBusy()) {

                    shooterTimer.resetTimer();
                    pathState = 4;
                }

                break;



            case 4:

                stopShooter();
                break;
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
    public void shootLogic() {

        double READY_LOW = 3600;
        double READY_HIGH = 3750;

        double targeted2 = 3600.0;

        double kF2 = 0.68;
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

    //
    public void stopShooter () {

        ShooterMotor.setPower(0);
        AssistantShooter.setPower(0);
        Collector.setPower(0);
    }
}
