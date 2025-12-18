package org.firstinspires.ftc.teamcode.revamped.utils.Commands;

public class StackTraceHelper {
    public static StackTraceElement ofClass(Class<?> clazz) {
        // Grab the current thread stack trace
        StackTraceElement[] stack = Thread.currentThread().getStackTrace();
        // stack[0] = getStackTrace, stack[1] = this method, stack[2] = caller
        StackTraceElement caller = stack[2];
        // Create a new StackTraceElement using the provided class
        return new StackTraceElement(
                clazz.getSimpleName(),        // class name
                caller.getMethodName(),       // method name
                clazz.getSimpleName() + ".java", // file name
                caller.getLineNumber()        // line number
        );
    }
}
