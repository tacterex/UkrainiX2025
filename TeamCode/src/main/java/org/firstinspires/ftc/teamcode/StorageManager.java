package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import java.io.IOException;

public class StorageManager {
    private boolean internal_storage_available(){
        return true;
    }

    public static boolean save_calibration(int ticks){
        return true;
    }

    public static int load_calibration(){
        return 0;
    }
}
