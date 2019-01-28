package com.team2898.engine.motion

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator


class LatencyComp(val name: String, val hz: Int = 50) {
    var table = mutableListOf<Pair<Double, Double>>()

    fun put(title: Double, value: Double) {
        table.add(Pair(title, value))
        if (table.size == hz) table.removeAt(0)
    }

    fun getEstimate(currentTime: Double): Double {
        val x = mutableListOf<Double>()
        val y = mutableListOf<Double>()

        table.forEach {
            x.add(it.first)
            y.add(it.second)
        }
        return LinearInterpolator().interpolate(x.toDoubleArray(), y.toDoubleArray()).value(currentTime)
    }

}