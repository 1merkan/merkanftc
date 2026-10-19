package org.firstinspires.ftc.teamcode.swervely;

import android.text.TextUtils;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.LinkedHashMap;
import java.util.ArrayList;
import java.util.Objects;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

/*
 * SWERVELY CHECK PROTOCOL
 *
 * the laptop app reads this check from FtcDashboard telemetry packets. every key starts with
 * "check/". the dashboard turns every value into text, so the laptop parses the numbers itself.
 * the laptop depends on these keys, so dont rename them. adding new keys is fine.
 *
 * while the check runs, every loop:
 *   check/stage      what is happening right now, plain text: "waiting" (during init),
 *                    "battery at rest", "fl forward", "fl backward", ... , "all wheels".
 *                    "stopped" if stop was pressed before the check finished (sent once, the
 *                    results are not sent in that case)
 *   check/progress   0 to 1, how far through the check we are
 *   check/done       "0"
 *
 * when the check finishes, this whole block is sent every loop until stop is pressed:
 *   check/done                "1"
 *   check/stage               "done"
 *   check/progress            1
 *   check/motors              comma list of the tested motors in test order: "fl,fr,rl,rr" and
 *                             then the ExtraMotors. use it to find the per motor keys below
 *   check/<motor>/fwd_tps     average encoder speed with forward power, ticks per second.
 *                             positive when the encoder counts the right way
 *   check/<motor>/rev_tps     the same with backward power, should be negative
 *   check/<motor>/amps        average current the motor pulled while spinning (both ways), amps
 *   check/<motor>/encoder     "ok"        forward power made the encoder count up
 *                             "reversed"  the encoder counts the wrong way for this motor's direction
 *                             "dead"      the encoder barely counted (under DeadTps both ways).
 *                                         if amps is also near 0 the motor itself didnt run
 *                             "missing"   no motor with this name in the robot configuration,
 *                                         the numbers are 0
 *                             wheels use the driving program's directions. ExtraMotors use FORWARD,
 *                             so "reversed" on one of them means its code will need REVERSE
 *   check/battery/rest_v      battery voltage with nothing running, volts
 *   check/battery/min_v       lowest battery voltage while all four wheels spun together, volts
 *   check/pinpoint/status     "READY", or the first other status seen during the check
 *                             (CALIBRATING, NOT_READY, FAULT_BAD_READ, FAULT_NO_PODS_DETECTED,
 *                             FAULT_X_POD_NOT_DETECTED, FAULT_Y_POD_NOT_DETECTED, FAULT_IMU_RUNAWAY),
 *                             or "missing" when there is no "pinpoint" in the configuration
 *   check/pinpoint/updating   "yes" if the pinpoint's readings kept changing, "no" if they froze
 *   check/pinpoint/drift_in   how far the position wandered from where it started, inches.
 *                             the robot is on blocks, so this should stay near 0. -1 when missing
 *   check/pinpoint/drift_deg  how far the heading wandered from where it started, degrees.
 *                             -1 when missing
 *   check/loop/avg_ms         average time of one loop over the whole check, milliseconds
 *   check/loop/max_ms         the slowest single loop, milliseconds
 *   check/found               comma list of every other motor, servo and continuous servo in the
 *                             configuration. they were found but NOT moved or tested. "" if none
 */

/**
 * a full hardware check for the swervely laptop app. it shows up on the driver hub as "SwervelyCheck".
 *
 * put the robot on blocks so no wheel touches anything, then press init and start. it spins every
 * wheel motor by itself both ways, then all four together, while it watches the battery, the
 * pinpoint and how fast the program runs. at the end the laptop shows what passed and what didnt.
 *
 * only the four wheels and the motors listed in ExtraMotors ever move. anything else in the
 * configuration is reported as found but left alone.
 */
@Config
@TeleOp(name = "SwervelyCheck", group = "Swervely")
public class SwervelyCheck extends LinearOpMode {
    // how much power the wheel motors get during the check, from 0 to 1. kept low, the robot is on blocks
    public static double Power = 0.3;

    /*
     * how much power the ExtraMotors get, from 0 to 1.
     * lower than the wheels because mechanisms like lifts and arms can run into their own end stops.
     */
    public static double ExtraPower = 0.2;

    // seconds each motor spins in each direction, counting the spin-up below
    public static double SpinSeconds = 1.5;

    // seconds at the start of every spin that dont get measured, while the motor is still speeding up
    public static double SpinUpSeconds = 0.5;

    // seconds of sitting still at the start, to read the battery with nothing running
    public static double RestSeconds = 1.0;

    // an encoder slower than this (ticks per second) both ways counts as "dead"
    public static double DeadTps = 50;

    /*
     * mechanism motors to test, by their names in the robot configuration, separated by commas,
     * like "lift,intake". each one gets the same spin test as the wheels, at ExtraPower.
     * leave it empty while the robot only has wheels. a change here counts from the next init.
     */
    public static String ExtraMotors = "";

    // the wheel motors in test order, with the same directions as the driving program
    private static final String[] DriveNames = {"fl", "fr", "rl", "rr"};

    private static final DcMotorSimple.Direction[] DriveDirections = {
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.REVERSE,
    };

    private GoBildaPinpointDriver Pinpoint;
    private VoltageSensor Battery;

    // time since start and how long the whole check should take, for the progress bar
    private final ElapsedTime Clock = new ElapsedTime();
    private double TotalSeconds;

    // loop times over the whole check
    private long LastLoop;
    private double LoopTotal;
    private double LoopMax;
    private int Loops;

    // what the pinpoint did over the whole check
    private String PinpointStatus;
    private String LastStatus = "";
    private int LastReading;
    private int Readings;
    private int Changes;

    private double StartX;
    private double StartY;
    private double StartHeading;

    private double DriftInches;
    private double DriftDegrees;

    // what the last Spin measured
    private double Tps;
    private double Amps;
    private double Volts;
    private double MinVolts;

    /**
     * runs when you press init and start. finds the hardware, runs the whole check once, then
     * keeps sending the results to the laptop until you press stop.
     */
    @Override
    public void runOpMode() {
        // the wheels first, then the mechanism motors. a name listed twice is only tested once
        List<String> Names = new ArrayList<>(Arrays.asList(DriveNames));

        for (String Name : ExtraMotors.split(",")) {
            String Trimmed = Name.trim();

            if (!Trimmed.isEmpty() && !Names.contains(Trimmed)) Names.add(Trimmed);
        }

        // tryGet gives null instead of crashing when a name isnt in the configuration, so a missing
        // motor shows up as "missing" on the laptop instead of ending the whole check
        DcMotorEx[] Motors = new DcMotorEx[Names.size()];

        for (int Index = 0; Index < Motors.length; Index++) {
            Motors[Index] = hardwareMap.tryGet(DcMotorEx.class, Names.get(Index));

            if (Motors[Index] == null) continue;

            Motors[Index].setDirection(Index < DriveNames.length ? DriveDirections[Index] : DcMotorSimple.Direction.FORWARD);
            Motors[Index].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Motors[Index].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // everything else that could move. it gets listed so nothing is skipped without anyone noticing
        List<String> Found = new ArrayList<>();

        for (HardwareMap.DeviceMapping<?> Mapping : new HardwareMap.DeviceMapping<?>[]{hardwareMap.dcMotor, hardwareMap.servo, hardwareMap.crservo}) {
            for (Map.Entry<String, ?> Entry : Mapping.entrySet()) {
                if (!Names.contains(Entry.getKey()) && !Found.contains(Entry.getKey())) Found.add(Entry.getKey());
            }
        }

        Pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, "pinpoint");
        PinpointStatus = Pinpoint == null ? "missing" : "READY";

        // same pods as the driving program, so drift comes out in real inches. resetting also
        // recalibrates the gyro, which is why the robot should already sit still on its blocks
        if (Pinpoint != null) {
            Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            Pinpoint.resetPosAndIMU();
        }

        // every hub reads the same battery, so the first voltage sensor is enough
        Battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("Put the robot on blocks so no wheel touches anything, then press start");
        telemetry.addData("Will spin", TextUtils.join(", ", Names));
        telemetry.addData("Wont touch", Found.isEmpty() ? "nothing" : TextUtils.join(", ", Found));
        telemetry.update();

        Send("waiting", 0);

        waitForStart();

        // read the powers once, so changing them on the dashboard mid-check cant surprise anyone
        double WheelPower = Range.clip(Power, 0, 1);
        double MechanismPower = Range.clip(ExtraPower, 0, 1);

        TotalSeconds = RestSeconds + SpinSeconds * (2 * Motors.length + 1);
        Clock.reset();

        Map<String, Object> Results = new LinkedHashMap<>();
        List<String> Summary = new ArrayList<>();

        // 1. the battery with nothing running
        boolean Running = Spin("battery at rest", RestSeconds, 0, null);
        double RestVolts = Volts;

        // 2. each motor by itself, forward and then backward
        for (int Index = 0; Index < Motors.length && Running; Index++) {
            String Name = Names.get(Index);
            DcMotorEx Motor = Motors[Index];
            double MotorPower = Index < DriveNames.length ? WheelPower : MechanismPower;

            double Forward = 0;
            double Backward = 0;
            double MotorAmps = 0;

            if (Motor != null) {
                Running = Spin(Name + " forward", SpinSeconds, MotorPower, Motor, Motor);
                Forward = Tps;
                MotorAmps = Amps;

                Running = Running && Spin(Name + " backward", SpinSeconds, -MotorPower, Motor, Motor);
                Backward = Tps;
                MotorAmps = (MotorAmps + Amps) / 2;
            }

            String Encoder = Motor == null ? "missing" : Encoder(Forward, Backward);

            Results.put("check/" + Name + "/fwd_tps", Forward);
            Results.put("check/" + Name + "/rev_tps", Backward);
            Results.put("check/" + Name + "/amps", MotorAmps);
            Results.put("check/" + Name + "/encoder", Encoder);

            Summary.add(String.format("%s: %s, %.0f / %.0f ticks/s, %.2f A", Name, Encoder, Forward, Backward, MotorAmps));
        }

        // 3. all four wheels together, the hardest the battery gets pulled during the check
        Running = Running && Spin("all wheels", SpinSeconds, WheelPower, null, Arrays.copyOf(Motors, DriveNames.length));
        double LowestVolts = MinVolts;

        // every Spin already turned its motors off, so stopping here leaves nothing running
        if (!Running) {
            Send("stopped", Range.clip(Clock.seconds() / TotalSeconds, 0, 1));

            telemetry.addLine("Stopped before the check finished");
            telemetry.update();
            return;
        }

        // ponytail: 1 in 10 is a guess. a working pinpoint changes almost every read, a frozen one never does
        boolean Updating = Changes > 0 && Changes * 10 >= Readings;
        double LoopAverage = Loops > 0 ? LoopTotal / Loops : 0;

        Results.put("check/done", "1");
        Results.put("check/stage", "done");
        Results.put("check/progress", 1);
        Results.put("check/motors", TextUtils.join(",", Names));

        Results.put("check/battery/rest_v", RestVolts);
        Results.put("check/battery/min_v", LowestVolts);

        // still calibrating after the whole check means it never finished
        if (PinpointStatus.equals("READY") && LastStatus.equals("CALIBRATING")) PinpointStatus = LastStatus;

        Results.put("check/pinpoint/status", PinpointStatus);
        Results.put("check/pinpoint/updating", Updating ? "yes" : "no");
        Results.put("check/pinpoint/drift_in", Pinpoint == null ? -1 : DriftInches);
        Results.put("check/pinpoint/drift_deg", Pinpoint == null ? -1 : DriftDegrees);

        Results.put("check/loop/avg_ms", LoopAverage);
        Results.put("check/loop/max_ms", LoopMax);

        Results.put("check/found", TextUtils.join(",", Found));

        // the short version for the driver hub. it stays on screen because nothing updates it again
        telemetry.addLine("Check done");

        for (String Line : Summary) telemetry.addLine(Line);

        telemetry.addData("Battery", "%.2f V at rest, %.2f V lowest", RestVolts, LowestVolts);
        telemetry.addData("Pinpoint", "%s, %s, drift %.2f in, %.1f deg", PinpointStatus, Updating ? "updating" : "frozen", DriftInches, DriftDegrees);
        telemetry.addData("Loop", "%.1f ms average, %.1f ms worst", LoopAverage, LoopMax);
        telemetry.addData("Not tested", Found.isEmpty() ? "nothing" : TextUtils.join(", ", Found));
        telemetry.update();

        // the laptop might connect late or drop a packet, so the results keep coming until stop
        while (opModeIsActive()) {
            TelemetryPacket Packet = new TelemetryPacket();
            Packet.putAll(Results);

            FtcDashboard.getInstance().sendTelemetryPacket(Packet);
            sleep(50);
        }
    }

    /**
     * one step of the check: gives some motors a power for a while, and keeps watching the battery,
     * the pinpoint and the loop time the whole time. the motors are always back at 0 afterwards.
     * the results land in Tps, Amps, Volts and MinVolts.
     *
     * @param Stage what the laptop shows for this step
     * @param Seconds how long the step takes
     * @param MotorPower power for the spinning motors, from -1 to 1
     * @param Measured the motor whose speed and current get measured, or null for none
     * @param Spinning the motors that get the power (nulls are skipped)
     * @return false if stop was pressed during the step
     */
    private boolean Spin(String Stage, double Seconds, double MotorPower, DcMotorEx Measured, DcMotorEx... Spinning) {
        double TpsTotal = 0;
        double AmpTotal = 0;
        double VoltTotal = 0;

        int Samples = 0;
        int VoltSamples = 0;

        MinVolts = 0;

        for (DcMotorEx Motor : Spinning) if (Motor != null) Motor.setPower(MotorPower);

        ElapsedTime Timer = new ElapsedTime();

        while (opModeIsActive() && Timer.seconds() < Seconds) {
            Watch(Stage);

            double Voltage = Battery.getVoltage();

            VoltTotal += Voltage;
            MinVolts = VoltSamples == 0 ? Voltage : Math.min(MinVolts, Voltage);
            VoltSamples++;

            // the start of every spin is skipped, the motor is still speeding up then
            if (Measured != null && Timer.seconds() >= SpinUpSeconds) {
                TpsTotal += Measured.getVelocity();
                AmpTotal += Measured.getCurrent(CurrentUnit.AMPS);
                Samples++;
            }
        }

        for (DcMotorEx Motor : Spinning) if (Motor != null) Motor.setPower(0);

        Tps = Samples > 0 ? TpsTotal / Samples : 0;
        Amps = Samples > 0 ? AmpTotal / Samples : 0;
        Volts = VoltSamples > 0 ? VoltTotal / VoltSamples : 0;

        return opModeIsActive();
    }

    /**
     * the part every loop of the check shares: times the loop, reads the pinpoint, and tells the
     * laptop and the driver hub what is happening.
     *
     * @param Stage what the laptop shows for the current step
     */
    private void Watch(String Stage) {
        long Now = System.nanoTime();

        if (LastLoop != 0) {
            double Ms = (Now - LastLoop) / 1e6;

            LoopTotal += Ms;
            LoopMax = Math.max(LoopMax, Ms);
            Loops++;
        }

        LastLoop = Now;

        if (Pinpoint != null) WatchPinpoint();

        double Progress = Range.clip(Clock.seconds() / TotalSeconds, 0, 1);

        Send(Stage, Progress);

        telemetry.addData("Checking", Stage);
        telemetry.addData("Progress", "%.0f%%", Progress * 100);
        telemetry.update();
    }

    /**
     * reads the pinpoint once and keeps track of its status, whether its numbers still change,
     * and how far it thinks the robot moved. the robot is on blocks, so it should think it didnt.
     */
    private void WatchPinpoint() {
        Pinpoint.update();

        double X = Pinpoint.getPosX(DistanceUnit.INCH);
        double Y = Pinpoint.getPosY(DistanceUnit.INCH);
        double Heading = Pinpoint.getHeading(AngleUnit.DEGREES);

        // keep the first thing that went wrong, so a fault that clears up again still shows.
        // CALIBRATING isn't a fault: the pinpoint says it for a moment after the reset in init
        String Now = Pinpoint.getDeviceStatus().name();

        LastStatus = Now;

        if (PinpointStatus.equals("READY") && !Now.equals("CALIBRATING")) PinpointStatus = Now;

        // a frozen pinpoint sends the exact same numbers every time. a working one never does,
        // its own loop time and the gyro wobble a little even when nothing moves
        int Reading = Objects.hash(Pinpoint.getLoopTime(), Pinpoint.getEncoderX(), Pinpoint.getEncoderY(), X, Y, Heading);

        if (Readings == 0) {
            StartX = X;
            StartY = Y;
            StartHeading = Heading;
        } else if (Reading != LastReading) {
            Changes++;
        }

        LastReading = Reading;
        Readings++;

        DriftInches = Math.max(DriftInches, Math.hypot(X - StartX, Y - StartY));
        DriftDegrees = Math.max(DriftDegrees, Math.abs(AngleUnit.normalizeDegrees(Heading - StartHeading)));
    }

    /**
     * decides whether a motor's encoder works, from how fast it counted each way.
     *
     * @param Forward average ticks per second with forward power
     * @param Backward average ticks per second with backward power
     * @return "ok", "reversed" or "dead"
     */
    private static String Encoder(double Forward, double Backward) {
        if (Math.max(Math.abs(Forward), Math.abs(Backward)) < DeadTps) return "dead";

        return Forward - Backward > 0 ? "ok" : "reversed";
    }

    /**
     * tells the laptop which step is running and how far along the check is.
     *
     * @param Stage what the laptop shows
     * @param Progress 0 to 1
     */
    private static void Send(String Stage, double Progress) {
        TelemetryPacket Packet = new TelemetryPacket();

        Packet.put("check/stage", Stage);
        Packet.put("check/progress", Progress);
        Packet.put("check/done", "0");

        FtcDashboard.getInstance().sendTelemetryPacket(Packet);
    }
}
