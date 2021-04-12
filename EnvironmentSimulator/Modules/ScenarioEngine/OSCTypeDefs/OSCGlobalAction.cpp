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

using namespace scenarioengine;
using namespace STGeometry;

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

void SwarmTrafficAction::Start()
{
	LOG("SwarmTrafficAction Start");
	printf("IR: %f, SMjA: %f, SMnA: %f\n", innerRadius_, semiMajorAxis_, semiMinorAxis_);

	odrManager_ = roadmanager::Position::GetOpenDrive();
	OSCAction::Start();
}

void print_sols(char sols, double x1, double y1, double x2, double y2, bool polar) {
	switch(sols) {
        case 2: {
		    printf("Polar: %d, P1 = (%.2f, %.2f), P2 = (%.2f, %.2f)\n", polar, x1, y1, x2, y2);
		    break;
	    }
	    case 1: {
		    printf("Polar: %d, P1 = (%.2f, %.2f)\n", polar, x1, y1);
		    break;
	    }
	    default: {
		    printf("No point detected\n");
			break;
	    }
	}
}

void SwarmTrafficAction::Step(double dt, double simTime)
{
	LOG("SwarmTrafficAction Step");
	detectPoints();
	spawn(ellipse_.lower, centralObject_->pos_.GetLaneId(), 0);
	spawn(ellipse_.upper, 1, 1);
	despawn();
	
}

void SwarmTrafficAction::initRoadSegments() {

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
	char sols;
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
	}
	return true;
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