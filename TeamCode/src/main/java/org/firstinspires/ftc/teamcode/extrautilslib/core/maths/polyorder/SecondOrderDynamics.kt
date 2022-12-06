@file:Suppress("FINITE_BOUNDS_VIOLATION_IN_JAVA")

package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.polyorder

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.times
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.EULVector
import kotlin.math.*

open class SecondOrderDynamics<T : EULVector<*>>(frequency : Double, dampening : Double, reaction : Double, start : T) {

    private var prevInput: EULVector<*>
    private var output: EULVector<*>
    private var outputDerivative: EULVector<*>
    private val k1: Double
    private val k2: Double
    private val k3: Double
    private val _w : Double
    private val _z : Double
    private val _d : Double

    init {

        if (frequency == 0.0) throw ArithmeticException("Frequency must not be zero!")

        _w = 2 * PI * frequency
        _z = dampening
        _d = _w * sqrt(abs(_z * _z - 1))

        k1 = dampening / (PI * frequency)
        k2 = 1 / ((2 * PI * frequency) * (2 * PI * frequency))
        k3 = reaction * dampening / (2 * PI * frequency)

        prevInput = start
        output = start
        outputDerivative = output - output
    }

    open fun update(deltaTime: Double, input: T, input_derivative: T? = null): EULVector<*> {
        var inD: EULVector<*> = (input_derivative ?: ((input - prevInput) / deltaTime))

        val k1Stable : Double
        val k2Stable : Double
        
        if (_w * deltaTime < _z){
            k1Stable = k1
            k2Stable = max(k2, max((deltaTime * deltaTime) / 2 + (deltaTime * k1) / 2, deltaTime * k1))
        } else {
            val t1 = exp(-_z * _w * deltaTime)
            val alpha = 2 * t1 * (if (_z <= 1) cos(deltaTime * _d) else cosh(deltaTime * _d))
            val beta = t1 * t1
            val t2 = deltaTime / (1 + beta - alpha)
            k1Stable = (1 - beta) * t2
            k2Stable = deltaTime * t2
        }

        output += deltaTime * outputDerivative
        outputDerivative += deltaTime * (input + k3 * inD - output - k1Stable * outputDerivative) / k2Stable

        this.prevInput = input
        return this.output
    }
}


