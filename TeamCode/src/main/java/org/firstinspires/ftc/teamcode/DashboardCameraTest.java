package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "DashboardCameraTest", group = "Test")
public class DashboardCameraTest extends LinearOpMode {

    private VisionPortal visionPortal;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1) Get the webcam from the configuration.
        //    Make sure the name "Webcam 1" matches your RC config exactly.
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // 2) Build a VisionPortal using that webcam.
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .build();

        // 3) Get the dashboard instance.
        dashboard = FtcDashboard.getInstance();

        // 4) Start camera stream to dashboard at ~30 FPS.
        dashboard.startCameraStream(visionPortal, 30);

        telemetry.addLine("DashboardCameraTest initialized.");
        telemetry.addLine("Connect to: http://<RC_IP>:8080/dash");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // You don’t need to do anything else for streaming.
            telemetry.addLine("Streaming camera to FTC Dashboard...");
            telemetry.update();

            sleep(50);
        }

        // Optional: stop stream when OpMode ends
        dashboard.stopCameraStream();
    }
}
