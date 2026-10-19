package org.firstinspires.ftc.teamcode.swervely;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import org.json.JSONObject;
import org.json.JSONArray;

/*
 * SWERVELY PATH PROTOCOL
 *
 * the laptop app draws a path, writes it into SwervelyPaths.PathJson on the dashboard and then runs
 * this program. the laptop and its simulator depend on this format and these keys, so dont rename
 * them. adding new keys is fine.
 *
 * PathJson:
 *   {"start":{"x":8,"y":64,"h":0},
 *    "segments":[{"name":"to basket","points":[[30,64],[40,90]],
 *                 "heading":{"mode":"linear","start":0,"end":90}}]}
 *
 *   everything is in pedro coordinates: inches, the origin in the bottom-left field corner, x to
 *   the right, y up, headings in degrees, counterclockwise positive, 0 = facing +x.
 *
 *   start      where the robot sits when start is pressed
 *   segments   driven in order. each one starts where the one before it ended, the first at start
 *   name       only for people, ignored here
 *   points     the control points and then the end point. 1 point = a straight line, 2 or more =
 *              a bezier curve through the start, the control points and the end
 *   heading    {"mode":"linear","start":deg,"end":deg}  turns from start to end along the segment.
 *                                                       pedro always takes the short way round
 *              {"mode":"constant","deg":deg}             faces one way the whole segment
 *              {"mode":"tangent","reverse":bool}         faces along the path, backwards when
 *                                                       reverse is true. reverse can be left out
 *
 * telemetry, sent every loop from init until stop. the dashboard turns every value into text:
 *   X, Y, Heading    the robot in the FTC field frame the laptop's field view uses: inches from the
 *                    field centre. X = 72 - pedro y, Y = pedro x - 72, Heading = pedro heading + 90,
 *                    degrees, -180..180
 *   path/x, path/y   the robot in pedro coordinates, inches
 *   path/h           the robot's pedro heading, degrees, -180..180
 *   path/segment     0-based index of the segment being followed. -1 when nothing is being followed:
 *                    before start, after the end, after "Stop path" or when PathJson is broken
 *   path/count       how many segments the path has, 0 when PathJson couldnt be read
 *   path/busy        "1" while pedro is driving the path or still settling on its end point, else "0"
 *   path/done        "1" once following is over (the end was reached, or "Stop path"), else "0"
 *   path/error       what is wrong with PathJson, "" when it is fine
 *
 * the robot button "Stop path" stops following and brakes the wheels where the robot is.
 */

/**
 * drives a path drawn on the laptop, so trying a new path doesnt need the code uploaded again. it
 * shows up on the driver hub as "SwervelyPathRunner".
 *
 * put the robot on the start the laptop shows, press init and keep it still (the pinpoint
 * recalibrates its gyro), then press start.
 *
 * pedro has to be tuned for this to drive well: the drivetrain, the pinpoint and above all the
 * ForesightConfig in pedro/Constants (the Mecanum, Pinpoint and Foresight tuners). untuned it can
 * wobble, crawl or miss the end point.
 */
@Autonomous(name = "SwervelyPathRunner", group = "Swervely")
public class SwervelyPathRunner extends LinearOpMode {
    private Follower Pedro;

    // what PathJson turned into. Chain stays null when it couldnt be read, and then nothing moves
    private Path Chain;
    private Pose Start;
    private int Count;
    private String Error = "";

    private boolean Started;

    /**
     * runs when you press init. reads the path and sets pedro up, then drives the path once start
     * is pressed, sending where the robot is the whole time.
     */
    @Override
    public void runOpMode() {
        Read(SwervelyPaths.PathJson);

        // the same follower the rest of the pedro code builds. at the end of the path it keeps
        // the robot on the end point instead of letting it roll
        Pedro = Constants.createFollower(hardwareMap);
        Pedro.holdEnd.set(true);

        if (Chain != null) Pedro.setPose(Start);

        // 40 updates a second instead of 10, so the robot moves smoothly on the laptop's field
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);

        // a press runs inside sendTelemetryPacket, on this thread, so it never lands halfway
        // through a pedro update
        FtcDashboard.getInstance().addAction("Stop path", "Stops following the path and brakes the wheels where the robot is.", this::StopPath);

        // the robot is carried to its start during init, so keep showing where it is
        while (opModeInInit()) {
            Pedro.update();
            Send();
        }

        if (isStopRequested()) return;

        // the robot might have been bumped since init, and it is supposed to be on the start now
        if (Chain != null) {
            Pedro.setPose(Start);
            Pedro.follow(Chain);
            Started = true;
        }

        while (opModeIsActive()) {
            Pedro.update();
            Send();
        }
    }

    /**
     * the "Stop path" button on the laptop. pedro stops following and from then on only brakes
     * the wheels. ponytail: braking, not pedro's hold, because an untuned hold can push the robot
     * somewhere else. use Pedro.hold(Pedro.pose()) if holding against pushes is wanted.
     */
    private void StopPath() {
        Pedro.stop();
    }

    /**
     * turns PathJson into one pedro path made of all the segments. it never throws: anything wrong
     * lands in Error as a message someone can act on, and Chain stays null.
     *
     * @param Json the text from SwervelyPaths.PathJson
     */
    private void Read(String Json) {
        if (Json == null || Json.trim().isEmpty()) {
            Error = "PathJson is empty. Draw a path on the laptop and send it first";
            return;
        }

        // which part was being read, so the message says where the mistake is
        String Where = "path";

        try {
            JSONObject Root = new JSONObject(Json);

            Where = "start";
            JSONObject StartJson = Root.getJSONObject("start");
            Pose StartPose = new Pose(StartJson.getDouble("x"), StartJson.getDouble("y"), Math.toRadians(StartJson.getDouble("h")));
            Pose Current = StartPose;

            Where = "segments";
            JSONArray Segments = Root.getJSONArray("segments");

            if (Segments.length() == 0) throw new IllegalArgumentException("there are none");

            Path[] Pieces = new Path[Segments.length()];

            for (int Index = 0; Index < Pieces.length; Index++) {
                Where = "segment " + Index;
                JSONObject Segment = Segments.getJSONObject(Index);
                Where = "segment " + Index + " (\"" + Segment.optString("name") + "\")";

                JSONArray Points = Segment.getJSONArray("points");

                if (Points.length() == 0) throw new IllegalArgumentException("it has no points");

                // the segment's own start, then its control points, then its end
                Pose[] Poses = new Pose[Points.length() + 1];
                Poses[0] = Current;

                for (int Point = 0; Point < Points.length(); Point++) {
                    JSONArray XY = Points.getJSONArray(Point);
                    Poses[Point + 1] = new Pose(XY.getDouble(0), XY.getDouble(1));
                }

                Path Piece = Poses.length == 2 ? line(Poses[0], Poses[1]) : curve(Poses);

                // pedro divides by a segment's length, so one that ends where it starts would
                // drive on garbage instead of failing here
                if (!(Piece.curve.length() > 0.01)) throw new IllegalArgumentException("it ends where it starts");

                JSONObject Heading = Segment.getJSONObject("heading");
                String Mode = Heading.getString("mode");

                switch (Mode) {
                    case "linear":
                        Piece = Piece.linear(Math.toRadians(Heading.getDouble("start")), Math.toRadians(Heading.getDouble("end")));
                        break;
                    case "constant":
                        Piece = Piece.constant(Math.toRadians(Heading.getDouble("deg")));
                        break;
                    case "tangent":
                        Piece = Heading.optBoolean("reverse") ? Piece.reverseTangent() : Piece.tangent();
                        break;
                    default:
                        throw new IllegalArgumentException("heading mode \"" + Mode + "\" isnt linear, constant or tangent");
                }

                // pedro needs the curve's direction at both ends (to steer, and for a tangent
                // heading). a control point sitting right on an end leaves it no direction there
                // and pedro throws just after start, so try both ends now and say so at init
                try {
                    Piece.curve.tangent(0);
                    Piece.curve.tangent(1);
                    Piece.heading(0);
                    Piece.heading(1);
                } catch (IllegalArgumentException e) {
                    throw new IllegalArgumentException("a control point sits on its start or end, so pedro has no direction there. move it a little");
                }

                Pieces[Index] = Piece;
                Current = Poses[Poses.length - 1];
            }

            Chain = path(Pieces);
            Start = StartPose;
            Count = Pieces.length;
        } catch (Exception e) {
            Error = Where + ": " + (e.getMessage() != null ? e.getMessage() : e.toString());
        }
    }

    /**
     * tells the laptop and the driver hub where the robot is and how the path is going.
     */
    private void Send() {
        Pose Robot = Pedro.pose();

        double X = Robot.x();
        double Y = Robot.y();
        double Heading = AngleUnit.normalizeDegrees(Math.toDegrees(Robot.heading()));

        boolean Following = Pedro.following();

        // right after the last segment pedro's index is one past the end for a single loop
        int Segment = Following ? Math.min(Pedro.pathIndex(), Count - 1) : -1;

        // pedro's busy flag only clears while it follows or holds, so once stopped it is not busy
        boolean Busy = !Pedro.idle() && Pedro.isBusy();
        boolean Done = Started && !Following;

        TelemetryPacket Packet = new TelemetryPacket();

        Packet.put("X", 72 - Y);
        Packet.put("Y", X - 72);
        Packet.put("Heading", AngleUnit.normalizeDegrees(Heading + 90));

        Packet.put("path/x", X);
        Packet.put("path/y", Y);
        Packet.put("path/h", Heading);
        Packet.put("path/segment", Segment);
        Packet.put("path/count", Count);
        Packet.put("path/busy", Busy ? "1" : "0");
        Packet.put("path/done", Done ? "1" : "0");
        Packet.put("path/error", Error);

        FtcDashboard.getInstance().sendTelemetryPacket(Packet);

        if (!Error.isEmpty()) {
            telemetry.addLine("PathJson is broken, nothing will move");
            telemetry.addData("Error", Error);
        } else if (!Started) {
            telemetry.addData("Ready", "%d segments, start %.1f, %.1f, %.0f deg", Count, Start.x(), Start.y(), AngleUnit.normalizeDegrees(Math.toDegrees(Start.heading())));
        } else {
            telemetry.addData("Path", Done ? "done" : "segment " + (Segment + 1) + " of " + Count);
        }

        telemetry.addData("Pose", "%.1f in, %.1f in, %.1f deg", X, Y, Heading);
        telemetry.update();
    }
}
