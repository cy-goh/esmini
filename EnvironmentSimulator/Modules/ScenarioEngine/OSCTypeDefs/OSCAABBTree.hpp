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
#include "RoadManager.hpp"
#include <memory>
#include <vector>

using namespace roadmanager;
using std::shared_ptr;
using std::vector;

namespace aabbTree {

    class Triangle;
    class BBox;
    class Tree;

    typedef shared_ptr<Triangle> ptTriangle;
    typedef shared_ptr<BBox> ptBBox;
    typedef shared_ptr<Tree> ptTree;
    
    typedef vector<ptBBox> BBoxVec;

    class Point {
    public:
        double x, y;
        Point(double x_, double y_) : x(x_), y(y_) {};
        Point(Point const &pt) : x(pt.x), y(pt.y) {};
        Point() : x(0), y(0) {};
    };

    class Triangle {
    public:
        double sI, sF;
        Point a, b, c;
        Triangle() : geometry_(nullptr), sI(0), sF(0) {}
        Triangle(Geometry *geometry) : geometry_(geometry) {}
        ~Triangle() {}
        Geometry *geometry() { return geometry_; }
        bool collide(ptTriangle const triangle) const;
    private:
        Geometry *geometry_;

        Triangle(Triangle const &triangle);
    };

    class BBox {
    public:
        BBox(ptTriangle triangle);
        BBox(BBoxVec const &bboxes);
        BBox(BBoxVec::const_iterator start, BBoxVec::const_iterator end);
        ~BBox(){}
        Point blhCorner() const { return blhc_;     }
        Point urhCorner() const { return urhc_;     }
        ptTriangle triangle() { return triangle_; }
        inline bool collide(ptBBox const bbox) const;
        bool collide(BBox const &bbox) const;
        double inline midPointX() const;
        double inline midPointY() const;
    private:
        Point blhc_, urhc_;
        ptTriangle triangle_;

        void merge(BBoxVec::const_iterator start, BBoxVec::const_iterator end);
    };

    class Tree {
    public:
        Tree(){ nodeCount_ = 0; }
        ~Tree();
        void intersect(Tree const &tree) const;
        void build(BBoxVec &bboxes);
        unsigned long nodeCount() { return nodeCount_; }
    private:
        ptBBox bbox;
        vector<ptTree> childeren;
        unsigned long nodeCount_; // for debug;

        Tree(Tree const &tree);
        BBoxVec::iterator divide(BBoxVec::iterator const start, BBoxVec::iterator const end, ptBBox bbox);
        void __build(BBoxVec::iterator const start, const BBoxVec::iterator end);

        typedef struct {
            BBoxVec::iterator first, last;
            ptTree tree;
        } StackRecord;
    };
}