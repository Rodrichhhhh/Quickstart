package org.firstinspires.ftc.teamcode.mylib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

public class RobotPos2 {

    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // Odometry encoders
    private DcMotor leftOdo, rightOdo, midOdo;

    private IMU imu;

    // =========================
    // CONSTANTS
    // =========================

    private final double TICKS_PER_REV = 8192.0;
    private final double WHEEL_RADIUS = 2.54; // cm
    private final double GEAR_RATIO = 1.0;

    private final double FORWARD_OFFSET = -36; // cm

    private final double P = 3.0;

    // =========================
    // POSE
    // =========================

    private double x = -17;
    private double y = -14;
    private double theta = 0;

    private double lastTheta = 0;

    private int lastLeft, lastRight, lastMid;

    private double headingOffset = 0;

    // =========================
    // AUTO AIM
    // =========================

    private boolean autoAim = false;

    // =========================
    // BASKET MEMORY
    // =========================

    private double BASKET_X = 0;
    private double BASKET_Y = 0;

    private boolean memoryActive = false;
    private boolean lastYState = false;

    private double lastError = 0;
    private final double D = 0.8;
    private final double MIN_POWER = 0.08;

    private static final String MEMORY_FILE = "basket_memory.txt";

    private final List<ShotLine> shotLines = new ArrayList<>();

    private static class ShotLine {
        double x, y, angle;
        ShotLine(double x, double y, double angle) {
            this.x = x;
            this.y = y;
            this.angle = angle;
        }
    }

    // =========================
    // CONSTRUCTOR
    // =========================

    public RobotPos2(DcMotor leftFront, DcMotor leftBack,
                     DcMotor rightFront, DcMotor rightBack,
                     DcMotor leftOdo, DcMotor rightOdo, DcMotor midOdo,
                     IMU imu) {

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;

        this.leftOdo = leftOdo;
        this.rightOdo = rightOdo;
        this.midOdo = midOdo;

        this.imu = imu;

        imu.resetYaw();

        resetEncoders();

        loadBasketFromFile();
    }

    private void resetEncoders() {
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        midOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        midOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastLeft = leftOdo.getCurrentPosition();
        lastRight = rightOdo.getCurrentPosition();
        lastMid = midOdo.getCurrentPosition();
    }

    // =========================
    // UPDATE LOOP
    // =========================

    public void update(boolean autoAimButton, boolean stopButton, boolean yButton) {

        // Toggle memory
        if (yButton && !lastYState) {
            memoryActive = !memoryActive;
        }
        lastYState = yButton;

        if (stopButton) {
            autoAim = false;
            stopAllMotors();
            return;
        }

        updateHeading();
        updateOdometry();

        if (autoAimButton) autoAim = true;

        if (autoAim) autoAimMotors();
    }

    // =========================
    // HEADING (IMU ONLY)
    // =========================

    private void updateHeading() {

        int currentLeft = leftOdo.getCurrentPosition();
        int currentRight = rightOdo.getCurrentPosition();

        double dLeft = ticksToDistance(currentLeft - lastLeft);
        double dRight = ticksToDistance(currentRight - lastRight);

        double dThetaOdo = (dRight - dLeft) / 39;

        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;

        // Complementary filter
        theta += dThetaOdo;                         // integrate encoder rotation
        theta = 0.01 * theta + 0.99 * imuHeading;   // slowly correct drift
        theta = normalize(theta);

        lastLeft = currentLeft;
        lastRight = currentRight;
    }

    public void setFieldZero() {
        headingOffset =
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    // =========================
    // ODOMETRY (IMU HEADING)

    private void updateOdometry() {

        int currentLeft = leftOdo.getCurrentPosition();
        int currentRight = rightOdo.getCurrentPosition();
        int currentMid = midOdo.getCurrentPosition();

        double dLeft = ticksToDistance(currentLeft - lastLeft);
        double dRight = ticksToDistance(currentRight - lastRight);
        double dMid = ticksToDistance(-(currentMid - lastMid));

        lastLeft = currentLeft;
        lastRight = currentRight;
        lastMid = currentMid;

        // Robot-centric movement
        double forward = (dLeft + dRight) / 2.0;
        double strafe = dMid - (FORWARD_OFFSET * (theta - lastTheta));

        // Use midpoint heading for better integration
        double avgTheta = normalize((theta + lastTheta) / 2.0);

        x += forward * Math.cos(avgTheta) - strafe * Math.sin(avgTheta);
        y += forward * Math.sin(avgTheta) + strafe * Math.cos(avgTheta);

        lastTheta = theta;
    }

    private double ticksToDistance(double ticks) {
        return ticks * (2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO) / TICKS_PER_REV;
    }

    private double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    // =========================
    // AUTO AIM
    // =========================



    private void autoAimMotors() {

        // Compute target angle using intersection of lines only
        Point intersection = computeBestIntersection();

        // If we don't have enough data yet, don't turn
        if (intersection == null) {
            stopAllMotors();
            return;
        }

        double targetAngle = Math.atan2(intersection.y - y, intersection.x - x);
        double error = normalize(targetAngle - theta);

        double stopThreshold = Math.toRadians(1.2);

        if (Math.abs(error) < stopThreshold) {
            stopAllMotors();
            autoAim = false;
            lastError = 0;
            return;
        }

        // Derivative term
        double derivative = error - lastError;
        lastError = error;

        double turnPower = (P * error) + (D * derivative);

        // Slow down near target
        if (Math.abs(error) < Math.toRadians(10)) {
            turnPower *= 0.5;
        }

        // Minimum power to prevent stall
        if (Math.abs(turnPower) < MIN_POWER) {
            turnPower = Math.signum(turnPower) * MIN_POWER;
        }

        turnPower = Range.clip(turnPower, -0.6, 0.6);

        leftFront.setPower(turnPower);
        leftBack.setPower(turnPower);
        rightFront.setPower(-turnPower);
        rightBack.setPower(-turnPower);
    }

    private void stopAllMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    // =========================
    // TRIANGULATION
    // =========================

    public void recordShot() {

        if (!memoryActive) return;

        shotLines.add(new ShotLine(x, y, theta));

        if (shotLines.size() >= 2) {
            Point p = computeBestIntersection();
            if (p != null) {
                BASKET_X = p.x;
                BASKET_Y = p.y;
                saveBasketToFile();
            }
        }
    }

    private Point computeBestIntersection() {
        List<Point> pts = new ArrayList<>();

        for (int i = 0; i < shotLines.size(); i++) {
            for (int j = i + 1; j < shotLines.size(); j++) {
                Point p = intersectLines(shotLines.get(i), shotLines.get(j));
                if (p != null) pts.add(p);
            }
        }

        if (pts.isEmpty()) return null;

        double avgX = 0, avgY = 0;
        for (Point p : pts) {
            avgX += p.x;
            avgY += p.y;
        }

        return new Point(avgX / pts.size(), avgY / pts.size());
    }

    private Point intersectLines(ShotLine l1, ShotLine l2) {

        double dx1 = Math.cos(l1.angle);
        double dy1 = Math.sin(l1.angle);

        double dx2 = Math.cos(l2.angle);
        double dy2 = Math.sin(l2.angle);

        double det = dx1 * dy2 - dy1 * dx2;
        if (Math.abs(det) < 1e-6) return null;

        double t = ((l2.x - l1.x) * dy2 - (l2.y - l1.y) * dx2) / det;

        return new Point(
                l1.x + t * dx1,
                l1.y + t * dy1
        );
    }

    // =========================
    // FILE SAVE
    // =========================

    private void saveBasketToFile() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(MEMORY_FILE);
            PrintWriter writer = new PrintWriter(file);
            writer.println(BASKET_X);
            writer.println(BASKET_Y);
            writer.close();
        } catch (Exception ignored) {}
    }

    private void loadBasketFromFile() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(MEMORY_FILE);
            if (!file.exists()) return;

            BufferedReader reader = new BufferedReader(new FileReader(file));
            BASKET_X = Double.parseDouble(reader.readLine());
            BASKET_Y = Double.parseDouble(reader.readLine());
            reader.close();
        } catch (Exception ignored) {}
    }

    // =========================
    // GETTERS
    // =========================

    public double getX() { return x; }
    public double getY() { return y; }
    public double getTheta() { return theta; }
    public double getBasketX() { return BASKET_X; }
    public double getBasketY() { return BASKET_Y; }

    private static class Point {
        double x, y;
        Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
    public boolean isMemoryActive() {
        return memoryActive;
    }

}