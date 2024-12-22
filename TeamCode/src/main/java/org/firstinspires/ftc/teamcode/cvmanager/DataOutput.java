package org.firstinspires.ftc.teamcode.cvmanager;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class DataOutput {
    OpenCvCamera camera;
    AprilTagDetectionPipeline pipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    static final double tagSize = 0.1016;

    AprilTagDetection tagOfInterest = null;

    public DataOutput(WebcamName w, int id){
        camera = OpenCvCameraFactory.getInstance().createWebcam(w, id);
        pipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public boolean find_tag(int target){
        boolean tagFound = false;
        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

        if(currentDetections.size() != 0) {
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == target) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
        }
        return tagFound;
    }
}
