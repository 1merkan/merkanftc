package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * a test program that finds a weak or slow wheel motor. it shows up on the driver hub as "MotorCheck".
 *
 * it spins each of the four wheel motors by itself, one after another, all at the same power,
 * and measures how fast each one actually turned. all four should end up close to each other.
 * if one is clearly slower, that motor, its gearbox or its wheel is the problem.
 *
 * lift the robot so the wheels dont touch anything before pressing start.
 */
@Config
@TeleOp(name = "MotorCheck", group = "Test")
public class MotorCheck extends LinearOpMode {
    public static double Power = 0.3; // how much power each motor gets during the test (0 to 1)
    public static double Seconds = 2.0; // how long each motor spins

    /**
     * runs when you press init and start. spins the motors one at a time,
     * then shows the speed of each one on the driver hub and the dashboard.
     */
    @Override
    public void runOpMode() {
        // the motor names from the robot configuration, in the order they get tested
        String[] Names = {"fl", "fr", "rl", "rr"};

        // same directions as the driving program, so every motor spins its wheel forward
        DcMotorSimple.Direction[] Directions = {
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
        };

        DcMotorEx[] Motors = new DcMotorEx[4];
        double[] Speeds = new double[4];
        double[] Amps = new double[4];

        // set each motor up and reset its turn counter to zero
        for (int Index = 0; Index < 4; Index++) {
            Motors[Index] = hardwareMap.get(DcMotorEx.class, Names[Index]);
            Motors[Index].setDirection(Directions[Index]);
            Motors[Index].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Motors[Index].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Motors[Index].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Lift the wheels off the ground, then press start");
        telemetry.update();

        waitForStart();

        // spin one motor at a time and count how far it turned.
        // the speed is "ticks per second", where a tick is a tiny step the motor's sensor counts.
        // we also measure how much electric current the motor pulls (in amps). a motor that is
        // stuck, rubbing or has a stiff gearbox pulls more current than the others
        for (int Index = 0; Index < 4 && opModeIsActive(); Index++) {
            int Start = Motors[Index].getCurrentPosition();
            long Began = System.nanoTime();

            Motors[Index].setPower(Power);
            sleep(500);

            double AmpTotal = 0;
            int Samples = 0;

            while (opModeIsActive() && (System.nanoTime() - Began) / 1e9 < Seconds) {
                AmpTotal += Motors[Index].getCurrent(CurrentUnit.AMPS);
                Samples++;
                sleep(50);
            }

            Amps[Index] = Samples > 0 ? AmpTotal / Samples : 0;

            int End = Motors[Index].getCurrentPosition();
            double Elapsed = (System.nanoTime() - Began) / 1e9;

            Motors[Index].setPower(0);
            Speeds[Index] = (End - Start) / Elapsed;

            sleep(500);
        }

        // show the results. a negative number means that motor spun backwards
        TelemetryPacket Packet = new TelemetryPacket();

        for (int Index = 0; Index < 4; Index++) {
            telemetry.addData(Names[Index], "%.0f ticks/s  %.2f A", Speeds[Index], Amps[Index]);
            Packet.put(Names[Index], Speeds[Index]);
            Packet.put(Names[Index] + " amps", Amps[Index]);
        }

        telemetry.update();
        FtcDashboard.getInstance().sendTelemetryPacket(Packet);

        // keep the results on screen until stop is pressed
        while (opModeIsActive()) idle();
    }
}
