package org.firstinspires.ftc.teamcode.utils.extrautilslib.core.maths.vectors

operator fun Int.times(eulVector: EULVector<*>): EULVector<*> {
    return eulVector * (this as Double)
}

operator fun Float.times(eulVector: EULVector<*>): EULVector<*> {
    return eulVector * (this as Double)
}

operator fun Double.times(eulVector: EULVector<*>): EULVector<*> {
    return eulVector * this
}

