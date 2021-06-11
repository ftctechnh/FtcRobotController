package org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems;

import org.firstinspires.ftc.teamcode.DutchFTCCore.Robotconfig;

import android.os.Environment;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Calendar;
import java.util.HashMap;

public class LoggingSubsystem extends SubSystem {

    public enum logType{
        NORMAL,
        WARNING,
        ERROR
    }

    static public String teamname = Robotconfig.teamName;
    FileOutputStream fileOut = null;

    static public HashMap<Integer, FileWriter> filewriters;

    public static void StartLogging(Integer id) throws IOException {
        String FileName =  teamname + Calendar.getInstance().getTime().toString() + ".csv";
        File file = new File(getPublicAlbumStorageDir(teamname),FileName );
        if(filewriters == null){
            filewriters = new HashMap<Integer, FileWriter>();
        }
        filewriters.put(id,new FileWriter(file));
    }

    public  static File getPublicAlbumStorageDir(String albumName) {
        // Get the directory for the user's public pictures directory.
        File file = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), albumName);
        if (!file.mkdirs()) {

        }
        return file;
    }

    public static void StopLogging(int id) {
        try {
            filewriters.get(id).close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void Log(logType type, String message, int id) {

        try {
            filewriters.get(id).write("\n"+ type.toString() + "," + Calendar.getInstance().getTime() + "," + message);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void Start() {
        
    }
}
