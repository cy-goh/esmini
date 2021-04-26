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

#include "OSCGlobalAction.hpp"
#include "OSCSwarmTrafficGeometry.hpp"
#include <memory>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace scenarioengine;
using namespace STGeometry;
using aabbTree::BBoxVec;
using aabbTree::ptTriangle;
using aabbTree::ptBBox;
using std::make_shared;
using aabbTree::BBox;

void ParameterSetAction::Start()
{
	LOG("Set parameter %s = %s", name_.c_str(), value_.c_str());
	parameters_->setParameterValue(name_, value_);
	OSCAction::Start();
}

void ParameterSetAction::Step(double dt, double simTime)
{
	OSCAction::Stop();
}

void print_triangles(BBoxVec &vec, char const filename[]) {
	std::ofstream file;
	file.open(filename);
	for (auto const bbx : vec) {
		auto trPtr = bbx->triangle();
        auto pt = trPtr->a;
		file << pt.x << "," << pt.y;
		pt = trPtr->b;
		file << "," << pt.x << "," << pt.y;
		pt = trPtr->c;
		file << "," << pt.x << "," << pt.y << "\n";
	}
	file.close();
}

void print_bbx(BBoxVec &vec, char const filename[]) {
	std::ofstream file;
	file.open(filename);
	for (auto const bbx : vec) {
        auto pt = bbx->blhCorner();
		file << pt.x << "," << pt.y;
		pt = bbx->urhCorner();
		file << "," << pt.x << "," << pt.y << "\n";
	}
	file.close();
}

void printTree(aabbTree::Tree &tree, char filename[]) {
	std::ofstream file;
	file.open(filename);
    std::vector<aabbTree::ptTree> v1, v2;
	v1.clear(); v2.clear();
    
	if (tree.empty()) {
		file.close();
		return;
	}

	auto bbx = tree.BBox();
	file << bbx->blhCorner().x << "," << bbx->blhCorner().y << 
	        "," << bbx->urhCorner().x << "," << bbx->urhCorner().y << "\n";
	v1.insert(v1.end(), tree.Children().begin(), tree.Children().end());

	while (!v1.empty()) {
        for (auto const tr : v1) {
            if (!tr->empty()) {
				auto bbox = tr->BBox();
	            file << bbox->blhCorner().x << "," << bbox->blhCorner().y << 
	                    "," << bbox->urhCorner().x << "," << bbox->urhCorner().y << ",";
				v2.insert(v2.end(), tr->Children().begin(), tr->Children().end());
			}
		}
		file << "\n";
		v1.clear();
		v1 = v2;
		v2.clear();
	}

	file.close();
}

void SwarmTrafficAction::Start()
{
	LOG("SwarmTrafficAction Start");
	printf("IR: %f, SMjA: %f, SMnA: %f\n", innerRadius_, semiMajorAxis_, semiMinorAxis_);
    double x0, y0, x1, y1;
	paramEllipse(0, 0, 0, semiMajorAxis_, semiMinorAxis_, 0, x0, y0);
	paramEllipse(M_PI / 36, 0, 0, semiMajorAxis_, semiMinorAxis_, 0, x1, y1);
	
	odrManager_ = roadmanager::Position::GetOpenDrive();
    minSize_    = ceil(sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2)) * 100) / 100.0;
	if (minSize_ == 0) minSize_ = 1.0;
	
	aabbTree::ptTree tree = std::make_shared<aabbTree::Tree>();
	aabbTree::BBoxVec vec;
	vec.clear();
	createRoadSegments(vec);

	tree->build(vec);
	rTree = tree;
	OSCAction::Start();
}

void SwarmTrafficAction::Step(double dt, double simTime)
{
	LOG("SwarmTrafficAction Step");
	BBoxVec vec;
	vec.clear();
	createEllipseSegments(vec);
	aabbTree::Tree eTree;
	bool flag = false;
    eTree.build(vec);
    
	std::ofstream outfile;
    outfile.open("test.txt", std::ios_base::app);

	aabbTree::Candidates candidates;
	candidates.clear();
	rTree->intersect(eTree, candidates);
	
	for (auto ca : candidates) {
		auto bbx1 = ca.bbox1;
		outfile << bbx1->blhCorner().x << "," << bbx1->blhCorner().y << ","
		        << bbx1->urhCorner().x << "," << bbx1->urhCorner().y << ",";
		auto bbx2 = ca.bbox2;
		outfile << bbx2->blhCorner().x << "," << bbx2->blhCorner().y << ","
		        << bbx2->urhCorner().x << "," << bbx2->urhCorner().y << ",";	
	}
	outfile << "\n";

	std::vector<ptTriangle> triangle;
    triangle.clear();
	aabbTree::processCandidates(candidates, triangle);
	printf("N of candidate triangles: %d\n", triangle.size());

	for (auto tr : triangle) {
		auto a = tr->a;
		auto b = tr->b;
		auto c = tr->c;

		outfile << a.x << "," << a.y << "," << b.x << "," << b.y <<
		        "," << c.x << "," << c.y << ",";
	}
	outfile << "\n";

	EllipseInfo info;
	info.egoPos = centralObject_->pos_;
	info.SMjA = semiMajorAxis_;
	info.SMnA = semiMinorAxis_;

	Solutions sols;
	sols.clear();
	aabbTree::findPoints(triangle, info, sols);
	printf("N of candidates: %d\n", sols.size());

	for (auto sol : sols) {
		outfile << sol.x << "," << sol.y << ",";
	}

	outfile << "\n";

	outfile.close();
}

void SwarmTrafficAction::createRoadSegments(BBoxVec &vec) {
    for (int i = 0; i < odrManager_->GetNumOfRoads(); i++) {
        roadmanager::Road* road = odrManager_->GetRoadByIdx(i);
        for (size_t i = 0; i < road->GetNumberOfGeometries(); i++) {
			roadmanager::Geometry *gm = road->GetGeometry(i);
			switch (gm->GetType()) {
			    case gm->GEOMETRY_TYPE_UNKNOWN: {
                    break;
                }
			    case gm->GEOMETRY_TYPE_LINE: {
					auto const length = gm->GetLength();
                    for (double dist = gm->GetS(); dist < length;) {
						double ds = dist + minSize_;
						if (ds > length)
						    ds = length;
						double x0, y0, x1, y1, x2, y2, dummy, l;
						gm->EvaluateDS(dist, &x0, &y0, &dummy);
						gm->EvaluateDS(ds, &x1, &y1, &dummy);
						l = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
						x2 = (x1 + x0)/2 + l / 4.0;
						y2 = (y1 + y0)/2 + l / 4.0;
                        Point a(x0, y0), b(x1, y1), c(x2, y2);
						ptTriangle triangle = make_shared<Triangle>(gm);
						triangle->a = a;
						triangle->b = b;
						triangle->c = c;
			            ptBBox bbox = make_shared<BBox>(triangle);
						vec.push_back(bbox);
						dist = ds;
					}
                    break;
                }
			    case gm->GEOMETRY_TYPE_ARC: {
					break;
				}
			    case gm->GEOMETRY_TYPE_SPIRAL: {
                    break;
                }
			    case gm->GEOMETRY_TYPE_POLY3: {
                    break;
                }
			    case gm->GEOMETRY_TYPE_PARAM_POLY3: {
                    break;
                }
			}
		}
	}
}

void SwarmTrafficAction::createEllipseSegments(aabbTree::BBoxVec &vec) {
	double alpha = -M_PI / 72.0;
	double dAlpha = M_PI / 36.0;
	auto pos = centralObject_->pos_;
	double x0, y0, x1, y1, x2, y2;
	while (alpha < (2 * M_PI - M_PI / 72.0)) {

		double da = alpha + dAlpha;
		if (da > 2 * M_PI - M_PI / 72.0)
		    da = 2 * M_PI - M_PI / 72.0;

		paramEllipse(alpha , pos.GetX(), pos.GetY(), semiMajorAxis_, semiMinorAxis_, pos.GetH(), x0, y0);
	    paramEllipse(da, pos.GetX(), pos.GetY(), semiMajorAxis_, semiMinorAxis_, pos.GetH(), x1, y1);

		double dPx, dPy, theta0, theta1, r0x, r0y, r1x, r1y, s;
		dPx = x1 - x0;
		dPy = y1 - y0;
        theta0 = angleTangentEllipse(semiMajorAxis_, semiMinorAxis_, alpha, pos.GetH());
		theta1 = angleTangentEllipse(semiMajorAxis_, semiMinorAxis_, da, pos.GetH());
		
		r0x = cos(theta0);
		r0y = sin(theta0);
        r1x = cos(theta1);
		r1y = sin(theta1);

		s = (dPy * r1x - dPx * r1y) / (r1x * r0y - r0x * r1y);

		x2 = x0 + s * r0x;
		y2 = y0 + s * r0y;

		ptTriangle triangle = make_shared<Triangle>();
		triangle->a = Point(x0, y0);
		triangle->b = Point(x1, y1);
		triangle->c = Point(x2, y2);
		vec.push_back(make_shared<BBox>(triangle));
        
		alpha = da;
	}
}

void SwarmTrafficAction::spawn(pointRef &pRef, int lane, double hdg_offset) {
	// Ensure spawnable point and some distance between two spawned veichles
	if (pRef.segmentIdx == -1 || (pRef.last && (abs(pRef.pos.GetS() - pRef.last->pos_.GetS()) < 20)))
	    return;
		
	Vehicle* vehicle = new Vehicle();
	vehicle->pos_.SetInertiaPos(pRef.pos.GetX(), pRef.pos.GetY(), centralObject_->pos_.GetH() + hdg_offset * M_PI, true);
	vehicle->pos_.SetLanePos(vehicle->pos_.GetTrackId(), lane, vehicle->pos_.GetS(), 0);
	vehicle->SetSpeed(centralObject_->GetSpeed());
	vehicle->controller_     = 0;
	vehicle->model_filepath_ = centralObject_->model_filepath_;
	int id                   = entities_->addObject(vehicle);
	vehicle->name_           = std::to_string(id);
	pRef.last             = vehicle;
	vehiclesId_.push_back(id);
}

bool SwarmTrafficAction::detectPoints() {
	/*char sols;
	pointInfo pt1, pt2;
	roadmanager::Position pos;

	roadmanager::Road* road = odrManager_->GetRoadByIdx(0);
    
	pt1.road = pt2.road = road; // Now just working with only one road
	for (size_t i = 0; i < road->GetNumberOfGeometries(); i++) {
		roadmanager::Geometry *geometry = road->GetGeometry(i);
		sols = lineIntersect(centralObject_->pos_, 
		                     static_cast<roadmanager::Line*>(geometry), 
							 semiMajorAxis_, semiMinorAxis_, 
							 &pt1.x, &pt1.y, &pt2.x, &pt2.y);
		switch (sols) {
			case 2:
			    pt1.segmentIdx = pt2.segmentIdx = i;
				break;
			case 1:
			    pt1.segmentIdx = i;
				break;
		}
	}

	switch(sols) {
		case 2: { 
			printf("Detected 2 points\n");

			pointRef &lower = ellipse_.lower;
			pointRef &upper = ellipse_.upper; 

			lower = pt1;
			upper = pt2;

			if (lower.pos.GetS() > upper.pos.GetS()) std::swap(lower, upper);
			break;
		}
		case 1: { 
			printf("Detected 1 points\n");
		    pointRef &ptRef = ellipse_.lower;
			ptRef = pt1;

			roadmanager::Position pos_ = centralObject_->pos_;

			if (ptRef.pos.GetS() > pos_.GetS()) {
				ellipse_.upper            = pt1;
				ellipse_.lower.segmentIdx = -1;
			} else {
				ellipse_.upper.segmentIdx = -1;
			}
			break;
		}
		default:
		    return false;
	}*/
	return false;
}

void SwarmTrafficAction::despawn() {
	auto idPtr = vehiclesId_.begin();
	bool increase = true;
	roadmanager::Position cPos = centralObject_->pos_;
	while (idPtr < vehiclesId_.end()) {
		Object *vehicle = entities_->GetObjectById(*idPtr);
		roadmanager::Position vPos = vehicle->pos_;
		auto e = ellipse(cPos.GetX(), cPos.GetY(), cPos.GetH(), semiMajorAxis_, semiMinorAxis_, vPos.GetX(), vPos.GetY());
		if (e > 0.001)
		{
            entities_->removeObject(vehicle->name_);
			delete vehicle;
			idPtr = vehiclesId_.erase(idPtr);
			increase = false;
		}

		if (increase) ++idPtr;
		increase = true;
	}
}