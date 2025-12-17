package org.firstinspires.ftc.teamcode.revamped.utils.Continuations;

import androidx.annotation.Nullable;

import dev.frozenmilk.dairy.mercurial.continuations.Continuation;

public interface SimpleContinuation extends Continuation {
    @Nullable
    @Override
    default StackTraceElement[] getStackTrace() {
        return new StackTraceElement[]{StackTraceHelper.ofClass(this.getClass())};
    };
}
