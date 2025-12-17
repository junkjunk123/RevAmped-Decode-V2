package org.firstinspires.ftc.teamcode.revamped.mechanisms.intake;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import dev.frozenmilk.dairy.mercurial.continuations.Continuation;

public class IntakeThread implements Continuation {
    @Nullable
    @Override
    public StackTraceElement[] getStackTrace() {
        return new StackTraceElement[0];
    }

    @NonNull
    @Override
    public Continuation apply() {
        return null;
    }
}
