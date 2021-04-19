/* 
 * esmini - Environment Simulator Minimalistic 
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * 
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#pragma once
#include <cmath>
#include "RoadManager.hpp"
#include "OSCAABBTree.hpp"
#include <functional>
#include <vector>

namespace STGeometry {

#define M(hdg) tan(hdg)
#define Q(x0, y0, hdg0) ((y0) - M(hdg) * (x0))

    typedef struct {
        double SMjA;
        double SMnA;
        roadmanager::Position egoPos;
    } EllipseInfo;

    typedef std::function<double(double)> DDProc;
    typedef std::vector<aabbTree::Point> Solutions;

    double inline ellipse(
        double h,
        double k,
        double A,
        double SMjA,
        double SMnA,
        double x,
        double y
    ) {
        double e1, e2;
        e1 = ((x - h) * cos(A) + (y - k) * sin(A)) / SMjA;
        e2 = ((x - h) * sin(A) - (y - k) * cos(A)) / SMnA;
        return pow(e1, 2) + pow(e2, 2) - 1;
    }

    /*
     * Line considered in the form y = mx + q
     * @m: angular coefficient of the line
     * @hdg: orientation of ellipses
     * @SMjA semi-major axes
     * @SMnA semi-minor axes
     */
    static inline double A(double m, double hdg, double SMjA, double SMnA) {
        double f1, f2, cos_, sin_;
        cos_ = cos(hdg);
        sin_ = sin(hdg);

        f1 = pow((m * sin_ + cos_) / SMjA, 2);
        f2 = pow((-m * cos_ + sin_) / SMnA, 2);
        return f1 + f2;
    }
    
    /*
     * Line considered in the form y = mx + q, where m = tan(theta)
     * @m: angular coefficient of the line
     * @q: y intercept of the line
     * @h: x-position of the center of the ellipses
     * @k: y-position of the center of the ellipses
     * @hdg: orientation of ellipses
     * @SMjA semi-major axes
     * @SMnA semi-minor axes
     */
    static inline double B(double m, double q, double h, double k, double hdg, double SMjA, double SMnA) {
        double f1, f2, cos_, sin_;
        cos_ = cos(hdg);
        sin_ = sin(hdg);

        f1   = 2 * (-h * cos_ + (q - k) * sin_) * (m * sin_ + cos_) / pow(SMjA, 2);
        f2   = 2 * (-h * sin_ - (q - k) * cos_) * (-m * cos_ + sin_) / pow(SMnA, 2);
        return f1 + f2;
    }

    /*
     * Line considered in the form y = mx + q, where m = tan(hdg)
     * @q: y intercept of the line
     * @h: x-position of the center of the ellipses
     * @k: y-position of the center of the ellipses
     * @hdg: orientation of ellipses
     * @SMnA: semi-minor axes
     * @SMjA semi-major axes
     */
    static inline double C(double q, double h, double k, double hdg, double SMjA, double SMnA) {
        double cos_, sin_, f1, f2;
        cos_ = cos(hdg);
        sin_ = sin(hdg);

        f1 = pow((-h * cos_ + (-k + q) * sin_) / SMjA, 2);
        f2 = pow((-h * sin_ - (-k + q) * cos_) / SMnA, 2);
        return f1 + f2 - 1;
    }

    static inline double vA(double e_hdg, double SMjA) {
        return pow(sin(e_hdg)/SMjA, 2);
    }

    static inline double vB(double x, double h, double k, double e_hdg, double SMjA) {
        return 2 * (-pow(sin(e_hdg), 2) * k) / pow(SMjA, 2);
    }

    static inline double vC(double x, double h, double k, double e_hdg, double SMjA, double SMnA) {
        double sin_e, f1, f2;
        sin_e = sin(e_hdg);

        f1 = pow((-sin_e * k) / SMjA, 2);
        f2 = pow(((x - h) * sin_e) / SMnA, 2);
        return f1 + f2 -1;
    }

    /*
     * Checks whether the intersection points found belong to the segment of road
     */
    static char checkRange(roadmanager::Line *line, char solsN, double* x1, double* y1, double* x2, double* y2) {
        double xmin, ymin, xmax, ymax, hdg;

        xmin = line->GetX();
        ymin = line->GetY();
        line->EvaluateDS(line->GetLength(), &xmax, &ymax, &hdg);

        if (xmin > xmax) std::swap(xmin, xmax);
        if (ymin > ymax) std::swap(ymin, ymax);

        switch (solsN) {
            case 2:
                if (!(xmin <= *x1 && *x1 <= xmax && ymin <= *y1 && *y1 <= ymax)) {
                   *x1 = *x2;
                   *y1 = *y2;
                   *x2 = *y2 = 0;
                   solsN = 1;
                } else if (!(xmin <= *x2 && *x2 <= xmax && ymin <= *y2 && *y2 <= ymax)) {
                    *x2 = *y2 = 0;
                    return 1;
                }
            case 1:
                if (!(xmin <= *x1 && *x1 <= xmax && ymin <= *y1 && *y1 <= ymax)) {
                    *x1 = *y1 = 0;
                    return 0;
                } else 
                    break;
            default:
                break;
        }
        return solsN;
    }

    static void checkRange(aabbTree::Triangle &triangle, Solutions &sols, size_t pos) {
        double xmin, ymin, xmax, ymax;
        xmin = triangle.a.x; 
        ymin = triangle.a.y;
        xmax = triangle.b.x;
        ymax = triangle.b.y; 

        if (xmin > xmax) std::swap(xmin, xmax);
        if (ymin > ymax) std::swap(ymin, ymax); 

        Solutions::const_iterator solsIter = sols.begin() + pos;
        while (solsIter < sols.end()) {
            double x, y;
            x = solsIter->x;
            y = solsIter->y;
            if (!(xmin <= x && x <= xmax && ymin <= y && y <= ymax))
                solsIter = sols.erase(solsIter);
            else 
                solsIter++;
        }   
    }

    /* 
     * Finds the intersection points between an ellipses and a stright piece of the road
     * The equation to find the x points of intersection is assumed to be in the form
     *   A x² + Bx² + C = 0
     * while to find the y points the line equation is used:
     *   y = m*x + q
     * 
     * The ellipse is considered to have the form:
     *   [(x - h)cos(A) + (y - k)sin(A)]²     [(x  - h)sin(A) - (y - k)cos(A)]²
     *  ---------------------------------- + ---------------------------------- = 1
     *                  a²                                   b²
     * Where:
     *   * x, y: coordinates of a point of the ellipse
     *   * h, k: coordinates of the center of the ellipses
     *   * A: orientation angle
     *   * a, b: semi-major axes and semi-minor axes    
     */
    char lineIntersect(roadmanager::Position pos, roadmanager::Line *line, double SMjA, double SMnA, double* x1, double* y1, double* x2, double* y2) {
        double _A, _B, _C, delta, x0, y0, hdg, sqrt_delta, m, q, h, k;
        *x1 = *x2 = *y1 = *y2 = 0;
        
        x0  = line->GetX();
        y0  = line->GetY();
        hdg = line->GetHdg();
        h   = pos.GetX();
        k   = pos.GetY();

        if (!(cos(hdg) <= SMALL_NUMBER)) {
            m   = M(hdg);
            q   = Q(x0, y0, hdg);
        
            hdg = pos.GetH();
            _A = A(m, hdg, SMjA, SMnA);
            _B = B(m, q, h, k, hdg, SMjA, SMnA);
            _C = C(q, h, k, hdg, SMjA, SMnA);
        } else {
            hdg = pos.GetH();
            _A = vA(hdg, SMjA);
            _B = vB(x0, h, k, hdg, SMjA);
            _C = vC(x0, h, k, hdg, SMjA, SMnA);
        }

        delta = pow(_B, 2) - 4 * _A * _C;
        if(delta > 0) {
            sqrt_delta = sqrt(delta);
            if (!(cos(hdg) <= SMALL_NUMBER)) {
                *x1 = (-_B - sqrt_delta) / (2 * _A);
                *x2 = (-_B + sqrt_delta) / (2 * _A);
                *y1 = m * (*x1) + q;
                *y2 = m * (*x2) + q;
            } else {
                *x1 = *x2 = x0;
                *y1 = (-_B - sqrt_delta) / (2 * _A);
                *y2 = (-_B + sqrt_delta) / (2 * _A);
            }
            return checkRange(line, 2, x1, y1, x2, y2);
        }
        else if (delta == 0) {
            if (!(cos(hdg) <= SMALL_NUMBER)) {
                *x1 = -_B / (2.0 * _A);
                *y1 = m * *x1 + q;
            } else {
                *x1 = x0;
                *y1 = -_B / (2.0 * _A);
            }
            return checkRange(line, 1, x1, y1, x2, y2);
        }
        else {
            return 0;
        }
    }

    /*
     * Finds the zeros of a function 'f' given te guess 
     * interval (a, b) and a tollerance 'delta'. The result
     * is saved in 'res' and the function returns true if a solution is found 
     */
    bool brent_zeros(int a, int b, double &res, double delta, DDProc f) {
        double fa, fb, fc, fs, c, d, s;
        bool flag;
        fa = f(a);
        fb = f(b);
        if (fa * fb >= 0) return false;
        if (abs(fa) < abs(fb)) std::swap(fa, fb);
        c = a;
        fc = f(c);
        flag = true;
        while (true) {
            if (fa  != fc && fb != fc) 
                s = (a * fb * fc) / ((fa - fb) * (fa - fc)) +
                    (b * fa * fc) / ((fb - fa) * (fb - fc)) +
                    (c * fa * fb) / ((fc - fa) * (fc - fb));
            else
                s = b - fb * (b - a) / (fb - fa);

            if (((3 * a + b) / 4 <= s && s <= b) ||
                (flag && abs(s - b) >= abs(b - c) / 2) ||
                (!flag && abs(s - b) >= abs(c - d) / 2) ||
                (flag && abs(b - c) < abs(delta)) ||
                (!flag && abs(c - d) < abs(delta)))
            {
                s = (a + b) / 2;
                flag = true;
            } else 
                flag = false;
            fs = f(s);
            d = c;
            c = b;
            if (fa * fs < 0)
                b = s;
            else
                a = s;
            if (abs(f(a)) < abs(f(b))) std::swap(a, b);
            
            if (f(s) == 0 || abs(b - a) <= SMALL_NUMBER) {
                res = s;
                return true;
            };
            fa = f(a);
            fb = f(b);
            fc = f(c);
        }
        return false;
    }

    // Any check to see if the spiral is a line or a curve must be performed outside this routine
    bool clothoidIntersect(aabbTree::Triangle &triangle, EllipseInfo eInfo, Solutions &sol) {
        double res;
        double h, k, A, SMjA, SMnA;
        roadmanager::Spiral *spiral = static_cast<roadmanager::Spiral*>(triangle.geometry());
        h = eInfo.egoPos.GetX();
        k = eInfo.egoPos.GetY();
        A = eInfo.egoPos.GetH();
        SMjA = eInfo.SMjA;
        SMnA = eInfo.SMnA;

        DDProc ellipseP = [h, k, A, SMjA, SMnA, spiral](double s) {
            double x, y, hdg, e;
            spiral->EvaluateDS(s, &x, &y, &hdg);
            return ellipse(h, k, A, SMjA, SMnA, x, y);
        };
        if (!brent_zeros(triangle.sI, triangle.sF, res, SMALL_NUMBER, ellipseP)) return false;
        double x, y, hdg;
        spiral->EvaluateDS(res, &x, &y, &hdg);
        sol.push_back(Point(x,y));
        checkRange(triangle, sol); // Maybe useless call
        return !sol.empty();
    }

}