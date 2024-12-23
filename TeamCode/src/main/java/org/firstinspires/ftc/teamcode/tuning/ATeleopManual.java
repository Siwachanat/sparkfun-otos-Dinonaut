package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

public class ATeleopManual extends LinearOpMode {
    public Servo S0;
    public Servo S1;
    public Servo S4;
    public Servo S5;
    private Servo Smid;
    private Servo SR;
    private Servo SL;
    private Servo Gripper;
    private DcMotor liftR;
    private DcMotor liftL;
    public int LiftReference;
    @Override
    public void runOpMode() throws InterruptedException {
        S0 = hardwareMap.get(Servo.class, "S0");
        S1 = hardwareMap.get(Servo.class, "S1");
        S5 = hardwareMap.get(Servo.class, "S5");
        S4 = hardwareMap.get(Servo.class, "S4");
        Smid = hardwareMap.get(Servo.class, "midGrip");
        SR = hardwareMap.get(Servo.class, "SR");
        SL = hardwareMap.get(Servo.class, "SL");
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        liftR = hardwareMap.get(DcMotor.class, "liftMotorR");
        liftL = hardwareMap.get(DcMotor.class, "liftMotorL");
        LiftReference = liftL.getCurrentPosition();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        S0.setPosition(0.1);
        S1.setPosition(0.15);
        S5.setPosition(0.6);
        double sfy = 0.5;
        double sfx = 0.5;
        double sfr = 0.5;
        waitForStart();
        S0.setPosition(0.1);
        while (opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_y) < 0.5)
                sfy = 0.2;
            else if (Math.abs(gamepad1.left_stick_y) < 0.7)
                sfy = 0.5;
            else
                sfy = 1;

            if (Math.abs(gamepad1.left_stick_x) < 0.5)
                sfx = 0.2;
            else if (Math.abs(gamepad1.left_stick_x) < 0.7)
                sfx = 0.5;
            else
                sfx = 1;

            if (Math.abs(gamepad1.right_stick_x) < 0.5)
                sfr = 0.2;
            else if (Math.abs(gamepad1.right_stick_x) < 0.7)
                sfr = 0.5;
            else
                sfr = 1;
//hand brake slowdown
            if (gamepad1.left_trigger > 0.5){
                sfy = 0.35;
                sfx = 0.35;
                sfr = 0.35;
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y*sfy,
                            -gamepad1.left_stick_x*sfx
                    ),
                    -gamepad1.right_stick_x*sfr
            ));

            drive.updatePoseEstimate();
            if (gamepad1.dpad_up) {
                S0.setPosition(1);
            } else if (gamepad1.dpad_down) {
                S1.setPosition(0.15);
                S0.setPosition(0.55);
                Thread.sleep(200);
                S0.setPosition(0.7);
            }
            if (gamepad1.dpad_left) {
                S4.setPosition(0.07);
            } else if (gamepad1.dpad_right) {
                S4.setPosition(0.5);
            }
            if (gamepad1.left_bumper) {
                S1.setPosition(0.15);
                S0.setPosition(1);
                Thread.sleep(50);
                S5.setPosition(0.15);
            } else if (gamepad1.right_bumper) {
                S5.setPosition(0.53);
                Thread.sleep(100);
                S1.setPosition(0.5);
                Thread.sleep(200);
                S1.setPosition(0.88);
                S4.setPosition(0.07);
            } else if (gamepad1.b) {
                S5.setPosition(gamepad1.right_trigger);
            }
            if (gamepad2.dpad_up){
                liftR.setPower(-1);
                liftL.setPower(1);
            } else if (gamepad2.dpad_down) {
                liftR.setPower(1);
                liftL.setPower(-1);
            }else {
                liftR.setPower(0.1);
                liftL.setPower(0.1);
            }
            if (gamepad2.right_bumper){
                Gripper.setPosition(0.81);
                S0.setPosition(0.7);
            } else if (gamepad2.left_bumper) {
                Gripper.setPosition(0.35);
                Smid.setPosition(0.15);
                liftR.setPower(0);
                liftL.setPower(0);

            }

            int posL = liftL.getCurrentPosition();
            telemetry.addData("Position bef", posL);
            telemetry.update();
            LiftReference = liftL.getCurrentPosition();
            if (posL > 2000) {
                Smid.setPosition(1.0);
                SR.setPosition(0.7);
                SL.setPosition(0.3);
            }else if (gamepad2.y){
                Gripper.setPosition(0.835);
                Thread.sleep(150);
                S0.setPosition(1);
                Smid.setPosition(0.6);
                SR.setPosition(1);
                SL.setPosition(0);
                liftR.setPower(-0.7);
                liftL.setPower(0.7);
                Thread.sleep(585);
            }else if (gamepad2.b){
                SR.setPosition(0.45);
                SL.setPosition(0.55);
                Gripper.setPosition(0.65);
            }else if (gamepad2.a){
                Gripper.setPosition(0.65);
                Smid.setPosition(0.32);
                SR.setPosition(0.31);
                SL.setPosition(0.69);
            }
            if (gamepad2.x){
                Gripper.setPosition(0.835);
                Thread.sleep(290);
                S0.setPosition(1);
                Smid.setPosition(1.0);
                Thread.sleep(100);
                SR.setPosition(0.7);
                SL.setPosition(0.3);
            }
//            telemetry.addData("x", drive.pose.position.x);
//            telemetry.addData("y", drive.pose.position.y);
//            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
