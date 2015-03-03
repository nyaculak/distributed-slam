#ifndef KinectGrabber_h
#define KinectGrabber_h

#define KINECT_HORIZONTAL_APERTURE DEG2RAD(57)
#define KINECT_DEPTH_STD_ERROR 0.01
#define KINECT_MAX_DEPTH 10 // meters

#include <mrpt/synch/CThreadSafeVariable.h>
#include <mrpt/hwdrivers/CKinect.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::opengl;
using namespace mrpt::system;

using namespace std;

struct TThreadParam
{
	TThreadParam() : quit(false), pushed_key(0) { }

	volatile bool   quit;
	volatile int    pushed_key;

	mrpt::synch::CThreadSafeVariable<CObservation3DRangeScanPtr> new_obs;     // RGB+D (+3D points)
};

void thread_grabbing(TThreadParam &p)
{
	try
	{
		CKinect  kinect;

		const std::string cfgFile = "kinect_calib.cfg";
		if (mrpt::system::fileExists(cfgFile))
		{
			cout << "Loading calibration from: "<< cfgFile << endl;
			kinect.loadConfig(mrpt::utils::CConfigFile(cfgFile), "KINECT");
		}
		else cerr << "Warning: Calibration file ["<< cfgFile <<"] not found -> Using default params.\n";

		// Open:
		cout << "Calling CKinect::initialize()...";
		kinect.initialize();
		cout << "OK\n";

		bool there_is_obs=true, hard_error=false;

		while (!hard_error && !p.quit)
		{
			// Grab new observation from the camera:
			CObservation3DRangeScanPtr  obs     = CObservation3DRangeScan::Create(); // Smart pointers to observations
			CObservationIMUPtr          obs_imu = CObservationIMU::Create();

			kinect.getNextObservation(*obs,*obs_imu,there_is_obs,hard_error);

			if (!hard_error && there_is_obs)
			{
				p.new_obs.set(obs);
			}

			if (p.pushed_key!=0)
			{
				switch (p.pushed_key)
				{
					case 27:
						p.quit = true;
						break;
				}

				// Clear pushed key flag:
				p.pushed_key = 0;
			}
		}
	}
	catch(std::exception &e)
	{
		cout << "Exception in Kinect thread: " << e.what() << endl;
		p.quit = true;
	}
}

class KinectGrabber
{
public:
  KinectGrabber(void);
  bool init();
  void grab(CObservation2DRangeScanPtr the_scan);
private:
  TThreadParam thrPar;
  mrpt::system::TThreadHandle thHandle;//= mrpt::system::createThreadRef(thread_grabbing ,thrPar);

  mrpt::opengl::CFrustumPtr   gl_frustum;// = mrpt::opengl::CFrustum::Create(0.2f, 5.0f, 90.0f, 5.0f, 2.0f, true, true );
  CObservation3DRangeScanPtr  last_obs;
};

KinectGrabber::KinectGrabber(void)
{
  thHandle = mrpt::system::createThreadRef(thread_grabbing ,thrPar);
  gl_frustum = mrpt::opengl::CFrustum::Create(0.2f, 5.0f, 90.0f, 5.0f, 2.0f, true, true );
}

bool KinectGrabber::init()
{
  cout << "Waiting for sensor initialization...\n";
  do {
    CObservation3DRangeScanPtr possiblyNewObs = thrPar.new_obs.get();
    if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP)
        return true;
    else 	mrpt::system::sleep(10);
  } while (!thrPar.quit);
  // Check error condition:
  if (thrPar.quit) return false;
}

void KinectGrabber::grab(CObservation2DRangeScanPtr the_scan)
{
  CObservation3DRangeScanPtr possiblyNewObs = thrPar.new_obs.get();
  //CObservation2DRangeScanPtr the_scan = CObservation2DRangeScan::Create();
  if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP &&
    (!last_obs  || possiblyNewObs->timestamp!=last_obs->timestamp))
  {
    // It IS a new observation:
    last_obs     = possiblyNewObs;

    // Convert ranges to an equivalent 2D "fake laser" scan:
    if (last_obs->hasRangeImage)
    {
      // Convert to scan:
      //the_scan = CObservation2DRangeScan::Create();
      const float vert_FOV = DEG2RAD(gl_frustum->getVertFOV());

      last_obs->convertTo2DScan(*the_scan, "KINECT_2D_SCAN", .5f*vert_FOV, .5f*vert_FOV );
        the_scan->sensorLabel = "LASER_SIM";
        the_scan->sensorPose.setFromValues(0.0,0,0.0);
        the_scan->maxRange = KINECT_MAX_DEPTH;
        the_scan->aperture = KINECT_HORIZONTAL_APERTURE;
        the_scan->stdError = KINECT_DEPTH_STD_ERROR;
    }
  }
}

#endif
