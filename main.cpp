/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CConfigFile.h>

#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CSensoryFrame.h>

#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CRobotSimulator.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CMultiMetricMap.h>

#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/hwdrivers/CJoystick.h>
#include <mrpt/hwdrivers/CKinect.h>

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CGridPlaneXY.h>

#include <mrpt/system/os.h>
#include <mrpt/synch/CThreadSafeVariable.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <iostream>
#include <cstdlib>
#include <unistd.h>

#include "SerialPort.h"
#include "KinectGrabber.h"
#include "RobotContainer.h"

#define SERIAL_PORT_ADDRESS "/dev/ttyACM0"

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;
using namespace boost;

CDisplayWindow3D		window("Display Window");
COccupancyGridMap2D		the_grid;
RobotContainer			master_robot(&window, &the_grid);
RobotContainer			slave_robot(&window, &the_grid);
CMultiMetricMap*		slam_map;
CJoystick				joystick;

StdVector_CPose2D		robot_path_GT, robot_path_ODO;
CPose2D					lastOdo, pose_start;

mrpt::opengl::CSetOfObjectsPtr		gl_grid, gl_slam_grid;
mrpt::opengl::CSetOfObjectsPtr		gl_robot;
mrpt::opengl::CPlanarLaserScanPtr 	gl_scan;
mrpt::opengl::CPointCloudPtr 		gl_path_GT;
mrpt::opengl::CPointCloudPtr 		gl_path_ODO;

void update_grid_map_3d()
{
	gl_grid->clear();
	the_grid.getAs3DObject( gl_grid );
}

/*
void RunSimulation()
{
	Start();


	KinectGrabber kinect_grabber;
	kinect_grabber.init();

	// Initialize serial stream

	SerialPort serial;
	if(!serial.start(SERIAL_PORT_ADDRESS, 9600)) return;
	// Begin
	sleep(5); // wait for arduino bootup
	serial.end_of_line_char('\n');
	
	//serial.write_some("BI010\r\n");
	//serial.write_some("PE011\r\n");
	//serial.write_some("RA01000\r\n");

	// Simulate robot
	the_robot.movementCommand(0,0);
	static CTicTac tictac;

	// Initiate map building
	CMetricMapBuilderRBPF::TConstructionOptions		rbpfMappingOptions;
	rbpfMappingOptions.loadFromConfigFile(CConfigFile("/home/nick/dev/mrpt/src/mrpt/share/mrpt/config_files/rbpf-slam/gridmapping_optimal_sampling.ini"),"MappingApplication");
	CMetricMapBuilderRBPF mapBuilder(rbpfMappingOptions);
	mapBuilder.options.verbose						 = true;
	mapBuilder.options.enableMapUpdating	 = true;
  mapBuilder.options.debugForceInsertion = false;

	// Store serial data
	
	double angle_rad;
	double x;
	double y;
	

	while(1)
	{
		double At = tictac.Tac();
		tictac.Tic();
		the_robot.simulateInterval(At);

		// Grab angle
		
		if(serial.angle_mutex_.try_lock()) {
			x = serial.get_x();
			y = serial.get_y();
			angle_rad = serial.get_angle();
			serial.angle_mutex_.unlock();
		}
		
		//cout << "X in: " << x << endl;
		//cout << "Y in: " << y << endl;
		//cout << "H in: " << angle_rad << endl << endl;


		// Simulate pose
		CPose2D p, odo_now;
		the_robot.getRealPose(p);
		the_robot.getOdometry(odo_now);

		// Measure pose
		
		CPose2D p(x/10, y/10, angle_rad);
		CPose2D odo_now(x/10, y/10, angle_rad);
		

		CObservation2DRangeScanPtr the_scan = CObservation2DRangeScan::Create();
		// Simulate scan
		the_scan->sensorLabel = "LASER_SIM";
		the_scan->sensorPose.setFromValues(0.0,0,0.0);
		the_scan->maxRange = 80;
		the_scan->aperture = LASER_APERTURE;
		the_scan->stdError = LASER_STD_ERROR;
		the_grid.laserScanSimulator(*the_scan, p, 0.2f, LASER_N_RANGES, LASER_STD_ERROR, 1, LASER_BEARING_STD_ERROR);

		// Grab scan from Kinect
		
		kinect_grabber.grab(the_scan);
		

		// Process Action
		CActionCollectionPtr acts = CActionCollection::Create();
		CActionRobotMovement2D	act;
		CActionRobotMovement2D::TMotionModelOptions	opts;
		opts.modelSelection = CActionRobotMovement2D::mmGaussian;
		CPose2D  Aodom = odo_now - lastOdo;
		lastOdo = odo_now;
		const TTimeStamp	tim_now = mrpt::system::now();
		act.computeFromOdometry(Aodom, opts);
		act.timestamp = tim_now;
		acts->insert(act);

		// Process Observation:
		CSensoryFramePtr sf = CSensoryFrame::Create();
		the_scan->timestamp = act.timestamp;
		sf->insert(CObservation2DRangeScanPtr(new CObservation2DRangeScan(*the_scan)));

		// Build map
		mapBuilder.processActionObservation(*acts, *sf);
		CMultiMetricMap	*mostLikMap = mapBuilder.mapPDF.getCurrentMostLikelyMetricMap();

		// Update GL Visualization:
		// :Robot
		gl_robot->setPose(p);
		// :Scan
		gl_scan->setScan(*the_scan);
		// :Path
		robot_path_GT.push_back(p);
		robot_path_ODO.push_back(odo_now);
		if (!robot_path_GT.empty())
		{
			gl_path_GT->insertPoint( robot_path_GT.rbegin()->x(), robot_path_GT.rbegin()->y(), 0 );
			CPose2D  this_odo = pose_start + *robot_path_ODO.rbegin();
			gl_path_ODO->insertPoint( this_odo.x(), this_odo.y(), 0 );
		}
		// :Map
		gl_slam_grid->clear();
		mostLikMap->getAs3DObject( gl_slam_grid );
		update_grid_map_3d();

		if (os::kbhit())
		{
			char c = os::getch();
			cout << "Key Pressed: " << c << endl;
			if (c == 'w')
				the_robot.movementCommand(1,0);
			else if (c == 'a')
				the_robot.movementCommand(0,1);
			else if (c == 's')
				the_robot.movementCommand(-1,0);
			else if (c == 'd')
				the_robot.movementCommand(0,-1);

			if (c == 'f')
				mostLikMap->saveMetricMapRepresentationToFile("map_out.png");

			if (c==27)
				break;
		}

		window.forceRepaint();
		usleep(3*10e3);
	}
}*/

void RunMultiRobotSimulation()
{
	window.setCameraAzimuthDeg(90);
	window.setCameraElevationDeg(90);

	gl_grid = CSetOfObjects::Create();
	the_grid.loadFromBitmapFile("virtual_map1.png", .0275/*12./(800.*3)*/);
	update_grid_map_3d();
	
	COpenGLScenePtr &theScene = window.get3DSceneAndLock();
		theScene->insert(gl_grid);
	window.unlockAccess3DScene();

	static CTicTac tictac;

	while(1)
	{
		double At = tictac.Tac();
		tictac.Tic();
		master_robot.simulate(At);
		slave_robot.simulate(At);

		if (os::kbhit())
		{
			char c = os::getch();
			master_robot.control(c);
			slave_robot.control(c, 'i', 'j', 'k', 'l', 'p');
		}

		master_robot.draw();
		slave_robot.draw();
		//master_robot.updateGLMap();
		//master_robot.updateGLSLAMMap();
		//update_grid_map_3d();
		//window.forceRepaint();
		usleep(3*10e3);
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		RunMultiRobotSimulation();
		return 0;
	} catch (std::exception &e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		cout << "Untyped exception caught" << endl;
		return -1;
	}
}
