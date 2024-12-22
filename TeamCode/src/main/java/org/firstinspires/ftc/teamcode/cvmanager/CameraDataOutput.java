package org.firstinspires.ftc.teamcode.cvmanager;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class CameraDataOutput {
    WebcamName webcam;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;

    public CameraDataOutput(WebcamName w, Size s){
        webcam = w;
        tagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(s)
                .build();

    }

    public AprilTagDetection findAprilTag(int id){
        ArrayList<AprilTagDetection> d = tagProcessor.getDetections();
        if(!d.isEmpty()){
            for(AprilTagDetection e : d){
                if(e.id == id) return e;
            }
        }
        return null;
    }
}
