package org.firstinspires.ftc.teamcode.logging;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.Calendar;
import java.util.GregorianCalendar;

/**
 * Provides basic logging functionality.
 * Modifications on 11/25/2017 to prevent exceptions when methods are called without initialization.
 * In addition, initialize() method modified so that if an exception occurs,
 * the bufferedWriter gets closed then set to null. The close() method is modified so that it sets bufferedWriter
 * to null before returning. With these changes, calls to the public methods of BetaLog (other than
 * initialize() and close() ) will be "do-nothing" statements if BetaLog has not been initialized,
 * if initialization has failed, or if it has already been closed.
 */

public class BetaLog {

    private static BufferedWriter bufferedWriter = null;
    private static final String defaultPath = "/sdcard/BetaLog.txt";
    private static ElapsedTime elapsedTime = null;

    private static boolean initialized = false;

    //Instantiate bufferedWriter and write the header. If this fails, close the bufferedWriter, then
    //set it to null.

    /**
     * Instantiate BufferedWriter using default path (in append mode), then write the header.
     * If this fails, close the buffered writer and set it to null.
     * @return True if successful, otherwise false.
     */
    public static boolean initialize(){
        initialized = true;
        elapsedTime = new ElapsedTime();
        GregorianCalendar gregorianCalendar = new GregorianCalendar();
        String header = String.format("BETA_LOG_INITIALIZED: %04d:%02d:%02d %02d:%02d:%02d:%03d",
                gregorianCalendar.get(Calendar.YEAR), gregorianCalendar.get(Calendar.MONTH)+1,
                gregorianCalendar.get(Calendar.DAY_OF_MONTH), gregorianCalendar.get(Calendar.HOUR_OF_DAY),
                gregorianCalendar.get(Calendar.MINUTE), gregorianCalendar.get(Calendar.SECOND),
                gregorianCalendar.get(Calendar.MILLISECOND));
        writeLine("");
        writeLine("");
        writeLine("");
        writeLine(header);
//        try {
//            bufferedWriter = new BufferedWriter(new FileWriter(defaultPath, true));
//            bufferedWriter.newLine();
//            bufferedWriter.newLine();
//            bufferedWriter.newLine();
//            writeLine(header);
//        }
//        catch(java.io.IOException e){
//            close();
//            return false;
//        }
        return true;
    }

    /**
     * Close the bufferedWriter, then set it to null.
     */
    public static void close(){
        initialized = false;
//        if (bufferedWriter != null){
//            try{
//                bufferedWriter.close();
//            }
//            catch (java.io.IOException e){
//                return;
//            }
//            finally {
//                bufferedWriter = null;
//            }
//        }
    }

    //Write a string to the log file, followed by a new line
    private static void writeLine(String string){
        if (!initialized) return;
        System.out.println(string);
//        if (bufferedWriter == null) return;
//        try{
//            bufferedWriter.write(string, 0, string.length());
//            bufferedWriter.newLine();
//        }
//        catch (java.io.IOException e){
//            return;
//        }
    }


    //Public logging methods, all of which are ultimately dependent on internalLog method:

    //Without TAG

    /**
     * Write argument list to log using format string.
     * @param format format string.
     * @param args arguments...
     */
    public static void d(String format, Object... args) { d(String.format(format, args)); }

    /**
     * Write message to log.
     * @param message message to write.
     */
    public static void d(String message) { internalLog(message); }

    //With TAG

    /**
     * Write TAG and arguments to log, using format string.
     * @param tag The TAG.
     * @param format Format string for args.
     * @param args arguments...
     */
    public static void dd(String tag, String format, Object... args) { dd(tag, String.format(format, args)); }

    /**
     * Write TAG and message to log.
     * @param tag
     * @param message
     */
    public static void dd(String tag, String message) {
        internalLog(tag + ": " + message);
    }

    //Write message to the log file, preceeded by elapsed time (since initialization) in seconds
    private static void internalLog( String message ){
        if (!initialized) return;
//        if (bufferedWriter == null) return;
        String string = String.format("  %.4f %s", elapsedTime.seconds(), message);
        writeLine(string);
//        try{
//            bufferedWriter.write(string, 0, string.length());
//            bufferedWriter.newLine();
//        }
//        catch (java.io.IOException e){
//            return;
//        }
    }

}
