
package com.team2898.engine.math.linear

import org.apache.commons.math3.linear.ArrayRealVector
import org.apache.commons.math3.linear.RealVector

open class Vector : ArrayRealVector {
    //constructor(data: DoubleArray) : super(data)
    constructor(vector: RealVector) : super(vector.toArray())
    constructor(vararg data: Double) : super(data)
}
