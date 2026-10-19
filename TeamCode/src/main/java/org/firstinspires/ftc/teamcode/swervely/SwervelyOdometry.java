package org.firstinspires.ftc.teamcode.swervely;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * SWERVELY ODOMETRY PROTOCOL
 *
 * the laptop app reads this from FtcDashboard telemetry packets, sent every loop from init until
 * stop. the dashboard turns every value into text, so the laptop parses the numbers itself.
 * the laptop depends on these keys, so dont rename them. adding new keys is fine.
 *
 *   odo/raw_x          raw count of the x (forward) pod's encoder, ticks, straight from the
 *                      pinpoint (getEncoderX). neither this program nor the SDK driver applies
 *                      PinpointSetup's directions or offsets to it
 *   odo/raw_y          the same for the y (strafe) pod (getEncoderY)
 *   odo/ticks_per_mm   how many ticks the pods count per millimetre rolled, the goBILDA 4-bar pod
 *                      resolution the driver uses (19.89436789). raw / this = millimetres rolled
 *   odo/heading_deg    the pinpoint's heading in degrees, counted past +-180 instead of wrapping,
 *                      so a full counterclockwise turn reads +360 and two read +720. 0 after a reset
 *   odo/x_in           position forward from where it was last reset, inches, with the current
 *                      PinpointSetup (getPosX)
 *   odo/y_in           position to the left of where it was last reset, inches (getPosY)
 *   odo/h_deg          the pinpoint's heading as it reports it, degrees, wrapped to -180..180
 *   odo/status         the pinpoint's status: "READY" when fine. otherwise NOT_READY, CALIBRATING
 *                      (for about a quarter second after a reset), FAULT_X_POD_NOT_DETECTED,
 *                      FAULT_Y_POD_NOT_DETECTED, FAULT_NO_PODS_DETECTED, FAULT_IMU_RUNAWAY,
 *                      FAULT_BAD_READ
 *   odo/x_dir          the x pod direction the pinpoint got, "FORWARD" or "REVERSED"
 *   odo/y_dir          the same for the y pod
 *   odo/x_offset_mm    PinpointSetup.XOffsetMm as the pinpoint got it, millimetres
 *   odo/y_offset_mm    PinpointSetup.YOffsetMm as the pinpoint got it, millimetres
 *   X, Y, Heading      the same as odo/x_in, odo/y_in and odo/h_deg, under the names the driving
 *                      program uses, so the field drawing and the pose readout work here too
 *
 * changing PinpointSetup on the dashboard takes effect on the next loop, without zeroing the
 * position. the robot button "Reset odometry" (or A on gamepad 1) zeroes the position and the
 * heading and recalibrates the gyro, so the robot has to sit still for it.
 */

/**
 * a passive odometry check for the swervely laptop app. it shows up on the driver hub as
 * "SwervelyOdometry".
 *
 * nothing moves. the wheels are left free to roll, you push the robot around by hand, and the
 * laptop watches what the pinpoint thinks happened. that is how the pod directions and offsets in
 * PinpointSetup get measured and checked before the other programs switch over to them.
 */
@TeleOp(name = "SwervelyOdometry", group = "Swervely")
public class SwervelyOdometry extends LinearOpMode {
    // the wheel motors, only so they can be set to roll freely
    private static final String[] DriveNames = {"fl", "fr", "rl", "rr"};

    private GoBildaPinpointDriver Pinpoint;

    // the heading counted past +-180, and the wrapped heading it was last built from
    private double UnwrappedHeading;
    private double LastHeading;

    /**
     * runs when you press init. sets the pinpoint up, then keeps sending what it measures until
     * you press stop. start changes nothing, it already measures during init.
     */
    @Override
    public void runOpMode() {
        // the robot gets pushed by hand, so the wheels have to coast instead of braking. nothing
        // here ever gives them power. tryGet so a missing motor cant stop the check
        for (String Name : DriveNames) {
            DcMotor Motor = hardwareMap.tryGet(DcMotor.class, Name);

            if (Motor != null) Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        String Applied = Settings();
        PinpointSetup.apply(Pinpoint);
        Reset();

        // 40 updates a second instead of 10, so the live readout on the laptop doesnt look choppy
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);

        // FtcDashboard.addAction needs dashboard newer than 0.6.0 — skipped for now

        // nothing ever moves, so there is no reason to wait for start
        while (!isStopRequested()) {
            // the laptop changes PinpointSetup live and then has the robot pushed again. a change
            // goes to the pinpoint straight away, and the position measured so far is kept
            String Current = Settings();

            if (!Current.equals(Applied)) {
                PinpointSetup.apply(Pinpoint);
                Applied = Current;
            }

            if (gamepad1.aWasPressed()) Reset();

            Pinpoint.update();

            double X = Pinpoint.getPosX(DistanceUnit.INCH);
            double Y = Pinpoint.getPosY(DistanceUnit.INCH);
            double Heading = Pinpoint.getHeading(AngleUnit.DEGREES);

            // the pinpoint wraps its heading at +-180. adding up the small steps between reads
            // keeps counting past that, so a full turn reads 360 instead of jumping back to 0
            UnwrappedHeading += AngleUnit.normalizeDegrees(Heading - LastHeading);
            LastHeading = Heading;

            int RawX = Pinpoint.getEncoderX();
            int RawY = Pinpoint.getEncoderY();

            String Status = Pinpoint.getDeviceStatus().name();
            String XDirection = PinpointSetup.Direction(PinpointSetup.XPodDirection).name();
            String YDirection = PinpointSetup.Direction(PinpointSetup.YPodDirection).name();

            TelemetryPacket Packet = new TelemetryPacket();

            Packet.put("odo/raw_x", RawX);
            Packet.put("odo/raw_y", RawY);
            Packet.put("odo/ticks_per_mm", PinpointSetup.TicksPerMm);
            Packet.put("odo/heading_deg", UnwrappedHeading);

            Packet.put("odo/x_in", X);
            Packet.put("odo/y_in", Y);
            Packet.put("odo/h_deg", Heading);
            Packet.put("odo/status", Status);

            Packet.put("odo/x_dir", XDirection);
            Packet.put("odo/y_dir", YDirection);
            Packet.put("odo/x_offset_mm", PinpointSetup.XOffsetMm);
            Packet.put("odo/y_offset_mm", PinpointSetup.YOffsetMm);

            Packet.put("X", X);
            Packet.put("Y", Y);
            Packet.put("Heading", Heading);

            FtcDashboard.getInstance().sendTelemetryPacket(Packet);

            // the short version for the driver hub
            telemetry.addLine("Push the robot by hand. A zeroes it (keep it still)");
            telemetry.addData("Pose", "%.2f in, %.2f in, %.1f deg", X, Y, UnwrappedHeading);
            telemetry.addData("Raw", "x %d, y %d ticks", RawX, RawY);
            telemetry.addData("Pods", "x %s, y %s", XDirection, YDirection);
            telemetry.addData("Pinpoint", Status);
            telemetry.update();
        }
    }

    /**
     * zeroes the position and the heading. the pinpoint also recalibrates its gyro, which takes
     * about a quarter second and needs the robot to sit still. used at init, by A and by the
     * "Reset odometry" button on the laptop.
     */
    private void Reset() {
        Pinpoint.resetPosAndIMU();

        // if the next read still shows the old heading, the step back to 0 after it cancels it out
        UnwrappedHeading = 0;
        LastHeading = 0;
    }

    /**
     * everything in PinpointSetup in one string, so a change from the dashboard is easy to spot.
     *
     * @return the directions and offsets joined together
     */
    private static String Settings() {
        return PinpointSetup.XPodDirection + "," + PinpointSetup.YPodDirection + "," + PinpointSetup.XOffsetMm + "," + PinpointSetup.YOffsetMm;
    }
}
