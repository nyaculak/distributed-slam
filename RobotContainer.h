#ifndef RobotContainer_h
#define RobotContainer_h

long   LASER_N_RANGES  = 361;
double LASER_APERTURE  = M_PI;
double LASER_STD_ERROR = 0.01;
double LASER_BEARING_STD_ERROR = DEG2RAD(0.05);

#include <mrpt/utils/CSerializable.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/base/link_pragmas.h>
#include <vector>

using namespace mrpt;
using namespace mrpt::gui;
using namespace std;

class ConstructionOptionContainer
{
public:
	ConstructionOptionContainer()
	{
		rbpfMappingOptions.loadFromConfigFile(CConfigFile("/home/nick/dev/mrpt/src/mrpt/share/mrpt/config_files/rbpf-slam/gridmapping_optimal_sampling.ini"),"MappingApplication");
	}
	CMetricMapBuilderRBPF::TConstructionOptions getMappingOptions()
	{
		return rbpfMappingOptions;
	}
private:
	CMetricMapBuilderRBPF::TConstructionOptions rbpfMappingOptions;
};

class RobotContainer
{
public:
	RobotContainer(CDisplayWindow3D*, COccupancyGridMap2D*);
	static int num_robots;
	static vector<RobotContainer*> robots;
	void simulate(double);
	void draw();
	void control(char);
	void updateGLMap();
	void updateGLSLAMMap();
	
//private:
	int robot_id;
	CRobotSimulator		robot_simulator;
	StdVector_CPose2D	robot_path_GT, robot_path_ODO;
	CPose2D				last_odo, pose_start;
	CPose2D				current_pose, current_odo;

	COccupancyGridMap2D* grid_ptr;

	// Map building
	ConstructionOptionContainer coc;
	CMetricMapBuilderRBPF mapBuilder;

	CObservation2DRangeScanPtr scan = CObservation2DRangeScan::Create();
	CMultiMetricMap	*mostLikMap;

	// OpenGL representation
	CDisplayWindow3D*					window_ptr;
	CDisplayWindow3D					slam_window;
	mrpt::opengl::CSetOfObjectsPtr		gl_slam_grid;
	mrpt::opengl::CSetOfObjectsPtr		gl_robot;
	mrpt::opengl::CPlanarLaserScanPtr 	gl_scan;
	mrpt::opengl::CPointCloudPtr 		gl_path_GT;
	mrpt::opengl::CPointCloudPtr 		gl_path_ODO;
};

int RobotContainer::num_robots = 0;


RobotContainer::RobotContainer(CDisplayWindow3D* window,
							   COccupancyGridMap2D* grid) :
	robot_simulator(0,0), 
	mapBuilder(coc.getMappingOptions()), 
	slam_window("SLAM OUTPUT")
{
	robot_id = num_robots++;
	robot_simulator.movementCommand(0, 0);
	robot_simulator.getRealPose(pose_start);
	robot_simulator.resetOdometry(CPose2D(0,0,0));
	last_odo = CPose2D(0,0,0);
	grid_ptr = grid;

	// odo errors:
	double Ax_err_bias   = 1e-5;//0;//.00001;
	double Ax_err_std    = 10e-4;
	double Ay_err_bias   = 1e-5;//0;//.00001;
	double Ay_err_std    = 10e-4;
	double Aphi_err_bias = 1e-5;//0;//5e-4;
	double Aphi_err_std  = 10e-4;

	robot_simulator.setOdometryErrors(true,
		Ax_err_bias, Ax_err_std,
		Ay_err_bias, Ay_err_std,
		Aphi_err_bias, Aphi_err_std);

	// Map building
	mapBuilder.options.verbose				 = false;	
	mapBuilder.options.enableMapUpdating	 = true;
	mapBuilder.options.debugForceInsertion	 = false;

	// OpenGL representation
	gl_robot = mrpt::opengl::stock_objects::RobotPioneer();
	gl_scan = mrpt::opengl::CPlanarLaserScan::Create();
	gl_robot->insert(gl_scan);

	gl_path_GT = mrpt::opengl::CPointCloud::Create();
	gl_path_GT->setColor(0,0,0, 0.7);
	gl_path_GT->setLocation(0,0, 0.01);
	gl_path_GT->setPointSize(3);

	gl_path_ODO = mrpt::opengl::CPointCloud::Create();
	gl_path_ODO->setColor(0,1,0, 0.7);
	gl_path_ODO->setLocation(0,0, 0.01);
	gl_path_ODO->setPointSize(2);

	gl_slam_grid = CSetOfObjects::Create();

	window_ptr = window;
	COpenGLScenePtr &scene = window_ptr->get3DSceneAndLock();
		scene->insert(gl_robot);
		scene->insert(gl_path_GT);
		scene->insert(gl_path_ODO);
	window_ptr->unlockAccess3DScene();

	slam_window.setCameraAzimuthDeg(90);
	slam_window.setCameraElevationDeg(90);

	COpenGLScenePtr &slam_scene = slam_window.get3DSceneAndLock();
		slam_scene->insert(gl_slam_grid);
	slam_window.unlockAccess3DScene();
	
	//robots.push_back(this);
}

void RobotContainer::simulate(double time) // get time from TicTac
{
	robot_simulator.simulateInterval(time);
	robot_simulator.getRealPose(current_pose);
	robot_simulator.getOdometry(current_odo);

	scan->sensorLabel = "LASER_SIM";
	scan->sensorPose.setFromValues(0.0,0,0.0);
	scan->maxRange = 80;
	scan->aperture = LASER_APERTURE;
	scan->stdError = LASER_STD_ERROR;
	grid_ptr->laserScanSimulator(*scan, current_pose, 0.2f, LASER_N_RANGES, LASER_STD_ERROR, 1, LASER_BEARING_STD_ERROR);

	// Process Action:
	CActionCollectionPtr acts = CActionCollection::Create();
	CActionRobotMovement2D	act;
	CActionRobotMovement2D::TMotionModelOptions	opts;
	opts.modelSelection = CActionRobotMovement2D::mmGaussian;
	CPose2D  Aodom = current_odo - last_odo;
	last_odo = current_odo;
	const TTimeStamp	tim_now = mrpt::system::now();
	act.computeFromOdometry(Aodom, opts);
	act.timestamp = tim_now;
	acts->insert(act);

	// Process Observation:
	CSensoryFramePtr sf = CSensoryFrame::Create();
	scan->timestamp = act.timestamp;
	sf->insert(CObservation2DRangeScanPtr(new CObservation2DRangeScan(*scan)));

	// Build map
	mapBuilder.processActionObservation(*acts, *sf);
	mostLikMap = mapBuilder.mapPDF.getCurrentMostLikelyMetricMap();
}

void RobotContainer::draw()
{
	// :Robot
	gl_robot->setPose(current_pose);
	// :Scan
	gl_scan->setScan(*scan);
	// :Path
	robot_path_GT.push_back(current_pose);
	robot_path_ODO.push_back(current_odo);
	if (!robot_path_GT.empty())
	{
		gl_path_GT->insertPoint( robot_path_GT.rbegin()->x(), robot_path_GT.rbegin()->y(), 0 );
		CPose2D  this_odo = pose_start + *robot_path_ODO.rbegin();
		gl_path_ODO->insertPoint( this_odo.x(), this_odo.y(), 0 );
	}
	// :Map
	gl_slam_grid->clear();
	mostLikMap->getAs3DObject(gl_slam_grid);
	slam_window.forceRepaint();
	window_ptr->forceRepaint();
}

void RobotContainer::updateGLMap()
{
	// :Robot
	gl_robot->setPose(current_pose);
	// :Scan
	gl_scan->setScan(*scan);
	// :Path
	robot_path_GT.push_back(current_pose);
	robot_path_ODO.push_back(current_odo);
	if (!robot_path_GT.empty())
	{
		gl_path_GT->insertPoint( robot_path_GT.rbegin()->x(), robot_path_GT.rbegin()->y(), 0 );
		CPose2D  this_odo = pose_start + *robot_path_ODO.rbegin();
		gl_path_ODO->insertPoint( this_odo.x(), this_odo.y(), 0 );
	}
}

void RobotContainer::updateGLSLAMMap()
{
	gl_slam_grid->clear();
	mostLikMap->getAs3DObject(gl_slam_grid);
	slam_window.forceRepaint();
}

void RobotContainer::control(char c)
{
	cout << "Key Pressed: " << c << endl;
	if (c == 'w')
		robot_simulator.movementCommand(1,0);
	else if (c == 'a')
		robot_simulator.movementCommand(0,1);
	else if (c == 's')
		robot_simulator.movementCommand(-1,0);
	else if (c == 'd')
		robot_simulator.movementCommand(0,-1);

	if (c == 'f')
		mostLikMap->saveMetricMapRepresentationToFile("map_out.png");
}

#endif
