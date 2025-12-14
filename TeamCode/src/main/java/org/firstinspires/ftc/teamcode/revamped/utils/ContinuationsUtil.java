package org.firstinspires.ftc.teamcode.revamped.utils;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import dev.frozenmilk.dairy.mercurial.continuations.Closure;
import dev.frozenmilk.dairy.mercurial.ftc.Context;

public class ContinuationsUtil {
    public static Closure infinite(Context context, Runnable infinite) {
        return sequence(
                waitUntil(context::inLoop),
                loop(exec(infinite))
        );
    }
}
