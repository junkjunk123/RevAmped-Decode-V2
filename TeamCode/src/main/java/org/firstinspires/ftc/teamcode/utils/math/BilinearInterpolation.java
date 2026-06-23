/*******************************************************************************
 * Copyright (c) 2010-2020 Haifeng Li. All rights reserved.
 *
 * Smile is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * Smile is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Smile.  If not, see <https://www.gnu.org/licenses/>.
 ******************************************************************************/

package org.firstinspires.ftc.teamcode.utils.math;

import org.firstinspires.ftc.teamcode.utils.math.calc.Vector2D;

import smile.interpolation.Interpolation2D;

/**
 * Bilinear interpolation in a two-dimensional regular grid. Bilinear
 * interpolation is an extension of linear interpolation for interpolating
 * functions of two variables on a regular grid. The key idea is to perform
 * linear interpolation first in one direction, and then again in the other
 * direction.
 *
 * @author Haifeng Li
 */
public class BilinearInterpolation implements Interpolation2D {

    private double[][] y;
    private LinearInterpolation x1terp, x2terp;

    /**
     * Constructor.
     */
    public BilinearInterpolation(double[] x1, double[] x2, double[][] y) {
        if (x1.length != y.length) {
            throw new IllegalArgumentException("x1.length != y.length");
        }

        if (x2.length != y[0].length) {
            throw new IllegalArgumentException("x2.length != y[0].length");
        }

        this.y = y;
        x1terp = new LinearInterpolation(x1, x1);
        x2terp = new LinearInterpolation(x2, x2);
    }

    @Override
    public double interpolate(double x1, double x2) {
        int i = x1terp.search(x1);
        int j = x2terp.search(x2);

        double t = (x1-x1terp.xx[i])/(x1terp.xx[i+1]-x1terp.xx[i]);
        double u = (x2-x2terp.xx[j])/(x2terp.xx[j+1]-x2terp.xx[j]);

        double yy = (1.-t)*(1.-u)*y[i][j] + t*(1.-u)*y[i+1][j] + (1.-t)*u*y[i][j+1] + t*u*y[i+1][j+1];

        return yy;
    }

    public Vector2D gradient(double x1, double x2) {
        final double EPS = 1e-3;

        int i = x1terp.search(x1);
        int j = x2terp.search(x2);

        double x1a = x1terp.xx[i];
        double x1b = x1terp.xx[i + 1];
        double x2a = x2terp.xx[j];
        double x2b = x2terp.xx[j + 1];

        // Nudge off discontinuities (grid boundaries).
        if (Math.abs(x1 - x1a) < EPS) x1 += EPS;
        else if (Math.abs(x1 - x1b) < EPS) x1 -= EPS;

        if (Math.abs(x2 - x2a) < EPS) x2 += EPS;
        else if (Math.abs(x2 - x2b) < EPS) x2 -= EPS;

        double dx1 = x1b - x1a;
        double dx2 = x2b - x2a;

        double t = (x1 - x1a) / dx1;
        double u = (x2 - x2a) / dx2;

        double y00 = y[i][j];
        double y10 = y[i + 1][j];
        double y01 = y[i][j + 1];
        double y11 = y[i + 1][j + 1];

        // ∂f/∂x1
        double gx =
                ((1.0 - u) * (y10 - y00) +
                        u * (y11 - y01)) / dx1;

        // ∂f/∂x2
        double gy =
                ((1.0 - t) * (y01 - y00) +
                        t * (y11 - y10)) / dx2;

        return new Vector2D(gx, gy);
    }

    @Override
    public String toString() {
        return "BiLinear Interpolation";
    }
}