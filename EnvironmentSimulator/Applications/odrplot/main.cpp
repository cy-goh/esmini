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

 /*
  * This application produces a raw data file to be used in Python for plotting the road network as simple graphs.
  *
  * This module reads an OpenDRIVE file and subsamples all roads within the road network. 
  * The resulting points are saved into a data file.
  * Use separate Python program xodr.py to plot the result.
  */

#include <cmath>
#include <iostream>
#include <fstream>
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include <sstream>
#include <iomanip> 
using namespace roadmanager;

std::string enum2string(roadmanager::LaneRoadMark::RoadMarkType rm){
	std::string ret("NONE");
	switch (rm) {
		case roadmanager::LaneRoadMark::RoadMarkType::NONE_TYPE:
			ret = "NONE";
			break;
		case roadmanager::LaneRoadMark::RoadMarkType::SOLID:
			ret = "SOLID";
			break;
		case roadmanager::LaneRoadMark::RoadMarkType::BROKEN:
			ret = "BROKEN";
			break;
		case roadmanager::LaneRoadMark::RoadMarkType::SOLID_SOLID:
			ret = "SOLID_SOLID";
			break;
		case roadmanager::LaneRoadMark::RoadMarkType::SOLID_BROKEN:
			ret = "SOLID_BROKEN";
			break;
		case roadmanager::LaneRoadMark::RoadMarkType::BROKEN_SOLID:
			ret = "BROKEN_SOLID";
			break;
		case roadmanager::LaneRoadMark::RoadMarkType::BROKEN_BROKEN:
			ret = "BROKEN_BROKEN";
			break;
		case roadmanager::LaneRoadMark::RoadMarkType::BOTTS_DOTS:
			ret = "BOTTS_DOTS";
			break;
		case roadmanager::LaneRoadMark::RoadMarkType::GRASS:
			ret = "GRASS";
			break;
		case roadmanager::LaneRoadMark::RoadMarkType::CURB:
			ret = "CURB";
			break;
	}

	return ret;
}

int main(int argc, char *argv[])
{
	std::string output_file_name = "track.csv";
	std::ofstream file;
	std::string sampling_step = "1.0";
	double step_length_target;
	static char strbuf[1024];
	
	if (argc < 2)
	{
		printf("Usage: ordplot openDriveFile.xodr [Output file, default=output.csv] [Sampling_step, default=1.0]\n");
		return -1;
	}
	else  
	{
		if (argc > 2)
		{
			output_file_name =  argv[2];
		}

		if (argc > 3)
		{
			sampling_step = argv[3];
		}
	}

	step_length_target = std::stod(sampling_step);

	try
	{
		if (Position::LoadOpenDrive(argv[1]) == false)
		{
			printf("Failed to open OpenDRIVE file %s\n", argv[1]);
			return -1;
		}
		file.open(output_file_name);
	}
	catch (std::exception& e) 
	{ 
		printf("exception: %s\n", e.what()); 
		return -1;
	}

	Position* pos = new Position();
	
	OpenDrive *od = Position::GetOpenDrive();

	for (int r = 0; r < od->GetNumOfRoads(); r++)
	{
		Road *road = od->GetRoadByIdx(r);

		for (int i = 0; i < road->GetNumberOfLaneSections(); i++)
		{
			LaneSection *lane_section = road->GetLaneSectionByIdx(i);
			double s_start = lane_section->GetS();
			double s_end = s_start + lane_section->GetLength();
			int steps = (int)((s_end - s_start) / step_length_target);
			double step_length = steps > 0 ? (s_end - s_start) / steps : s_end - s_start;

			for (int j = 0; j < lane_section->GetNumberOfLanes(); j++)
			{
				Lane *lane = lane_section->GetLaneByIdx(j);

				file << "lane, " << road->GetId() << ", " << i << ", " << lane->GetId() << (lane->IsDriving() ? ", driving" : ", no-driving") << std::endl;
				
				roadmanager::LaneRoadMark::RoadMarkType rmType;

				for (int k = 0; k < steps + 1; k++)
				{
					double s = MIN(s_end, s_start + k * step_length);
					float maxSOffset = -1.f;
					float widthRoadMark = -1;

					for (int l = 0; l < lane->GetNumberOfRoadMarks(); ++l) 
					{
						auto rm = lane->GetLaneRoadMarkByIdx(l);
						auto rmSOffset = rm->GetSOffset(); 
						if (rmSOffset <= s && rmSOffset > maxSOffset){
							rmType = rm->GetType();
							maxSOffset = rmSOffset;
							widthRoadMark = rm->GetWidth();
						}
					}

					// Set lane offset to half the lane width in order to mark the outer edge of the lane (laneOffset = 0 means middle of lane)
					pos->SetLanePos(road->GetId(), lane->GetId(), s, SIGN(lane->GetId())*lane_section->GetWidth(s, lane->GetId())*0.5, i);

					// Write the point to file
					snprintf(strbuf, sizeof(strbuf), "%f, %f, %f, %f, %s, %f\n", pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetH(), enum2string(rmType).c_str(), widthRoadMark);
					file << strbuf;
				}
			}
		}
	}

	for (int r = 0; r < od->GetNumOfRoads(); r++)
	{
		roadmanager::Road* road = od->GetRoadByIdx(r);

		for (size_t o = 0; o < road->GetNumberOfObjects(); o++)
		{
			roadmanager::RMObject* object = road->GetObject(o);
			std::cout << "GetType() " << object->GetType() << std::endl;
			//std::cout << "GetOrientation() " << object->GetOrientation() << std::endl;
			
			roadmanager::Position pos;
			pos.SetTrackPos(road->GetId(), object->GetS(), object->GetT());
			
			std::vector<std::array<double, 4>> coordinate_vec =  object->GetCoordinate();
			if(object->GetType()=="stopline")
			{
				for(std::array<double, 4> coordinate_array : coordinate_vec) {
					std::cout << "x " << coordinate_array[0] << " y " << coordinate_array[1] << " z "<< coordinate_array[2] << std::endl;
					file << std::setprecision(8) << coordinate_array[0] << ", " << std::setprecision(8) << coordinate_array[1] << ", " << std::setprecision(8) << coordinate_array[2] << std::endl;
				}
				//file << std::setprecision(8) << pos.GetX() << ", " << std::setprecision(8) << pos.GetY() << ", " << std::setprecision(8) << pos.GetZ() << std::endl;
				
			}
			
		}
	}

	file.close();

	delete pos;

	printf("Created %s using stepsize %.2f\n", output_file_name.c_str(), step_length_target);
	printf("To plot it, run EnvironmentSimulator/Applications/odrplot/xodr.py %s\n", output_file_name.c_str());

	return 0;
}

