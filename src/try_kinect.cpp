// #include <thread>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/openni_grabber.h>
// #include <pcl/common/time.h>

// #include <boost/filesystem.hpp>
// #include <boost/chrono.hpp>
// #include <boost/thread/thread.hpp> 

// #include <pcl/io/openni_grabber.h>
// #include <pcl/visualization/cloud_viewer.h>

// using namespace std::chrono_literals;

// class SimpleOpenNIProcessor
// {
// public:
//   void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
//   {
//     static unsigned count = 0;
//     static double last = pcl::getTime ();
//     if (++count == 30)
//     {
//       double now = pcl::getTime ();
//       std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
//       count = 0;
//       last = now;
//     }
//   }
  
//   void run ()
//   {
//     // create a new grabber for OpenNI devices
//     pcl::Grabber* interface = new pcl::OpenNIGrabber();

//     // make callback function from member function
//     std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
//       [this] (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) { cloud_cb_ (cloud); };

//     // connect callback function for desired signal. In this case its a point cloud with color values
//     boost::signals2::connection c = interface->registerCallback (f);

//     // start receiving point clouds
//     interface->start ();

//     // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
//     while (true)
//       std::this_thread::sleep_for(1s);

//     // stop the grabber
//     interface->stop ();
//   }
// };

//  class SimpleOpenNIViewer
//  {
//    public:
//      SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

//      void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
//      {
//        if (!viewer.wasStopped())
//          viewer.showCloud (cloud);
//      }

//      void run ()
//      {
//        boost::posix_time::seconds secTime(1);  
//         boost::this_thread::sleep(secTime); 
//        pcl::Grabber* interface = new pcl::OpenNIGrabber();

//        std::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
//          [this] (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) { cloud_cb_ (cloud); };

//        interface->registerCallback (f);

//        interface->start ();

//        while (!viewer.wasStopped())
//        {
//          boost::this_thread::sleep (boost::posix_time::seconds (1));

//             // boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
//         //  std::this_thread::sleep_for(1s); //alternative error boost::this_thread not declared
//        }

//        interface->stop ();
//      }

//      pcl::visualization::CloudViewer viewer;
//  };

//  int main ()
//  {
//    SimpleOpenNIViewer v;
//   //  SimpleOpenNIProcessor v;
//    v.run ();
//    return 0;
//  }









// Original code by Geoffrey Biggs, taken from the PCL tutorial in
// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php

// Simple OpenNI viewer that also allows to write the current scene to a .pcd
// when pressing SPACE.

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <iostream>


// #include <boost/chrono.hpp>
// #include <boost/thread/thread.hpp> 
// #include <boost/date_time/posix_time/posix_time.hpp>
// #include <boost/thread/thread.hpp> 

using namespace std;
using namespace pcl;

PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>); // A cloud that will store color info.
PointCloud<PointXYZ>::Ptr fallbackCloud(new PointCloud<PointXYZ>);    // A fallback cloud with just depth data.
boost::shared_ptr<visualization::CloudViewer> viewer;                 // Point cloud viewer object.
Grabber* openniGrabber;                                               // OpenNI grabber that takes data from the device.
unsigned int filesSaved = 0;                                          // For the numbering of the clouds saved to disk.
bool saveCloud(false), noColor(false);                                // Program control.

void
printUsage(const char* programName)
{
	cout << "Usage: " << programName << " [options]"
		 << endl
		 << endl
		 << "Options:\n"
		 << endl
		 << "\t<none>     start capturing from an OpenNI device.\n"
		 << "\t-v FILE    visualize the given .pcd file.\n"
		 << "\t-h         shows this help.\n";
}

// This function is called every time the device has new data.
void
grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
	if (! viewer->wasStopped())
		viewer->showCloud(cloud);

	if (saveCloud)
	{
		stringstream stream;
		stream << "inputCloud" << filesSaved << ".pcd";
		string filename = stream.str();
		if (io::savePCDFile(filename, *cloud, true) == 0)
		{
			filesSaved++;
			cout << "Saved " << filename << "." << endl;
		}
		else PCL_ERROR("Problem saving %s.\n", filename.c_str());

		saveCloud = false;
	}
}

// For detecting when SPACE is pressed.
void
keyboardEventOccurred(const visualization::KeyboardEvent& event,
					  void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		saveCloud = true;
}

// Creates, initializes and returns a new viewer.
boost::shared_ptr<visualization::CloudViewer>
createViewer()
{
	boost::shared_ptr<visualization::CloudViewer> v
	(new visualization::CloudViewer("OpenNI viewer"));
	v->registerKeyboardCallback(keyboardEventOccurred);

	return (v);
}

int
main(int argc, char** argv)
{
	if (console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return -1;
	}

	bool justVisualize(false);
	string filename;
	if (console::find_argument(argc, argv, "-v") >= 0)
	{
		if (argc != 3)
		{
			printUsage(argv[0]);
			return -1;
		}

		filename = argv[2];
		justVisualize = true;
	}
	else if (argc != 1)
	{
		printUsage(argv[0]);
		return -1;
	}

	// First mode, open and show a cloud from disk.
	if (justVisualize)
	{
		// Try with color information...
		try
		{
			io::loadPCDFile<PointXYZRGBA>(filename.c_str(), *cloudptr);
		}
		catch (PCLException e1)
		{
			try
			{
				// ...and if it fails, fall back to just depth.
				io::loadPCDFile<PointXYZ>(filename.c_str(), *fallbackCloud);
			}
			catch (PCLException e2)
			{
				return -1;
			}

			noColor = true;
		}

		cout << "Loaded " << filename << "." << endl;
		if (noColor)
			cout << "This cloud has no RGBA color information present." << endl;
		else cout << "This cloud has RGBA color information present." << endl;
	}
	// Second mode, start fetching and displaying frames from the device.
	else
	{
		openniGrabber = new OpenNIGrabber();
		if (openniGrabber == 0)
			return -1;
		boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
			boost::bind(&grabberCallback, _1);
		openniGrabber->registerCallback(f);
	}

	viewer = createViewer();

	if (justVisualize)
	{
		if (noColor)
			viewer->showCloud(fallbackCloud);
		else viewer->showCloud(cloudptr);
	}
	else openniGrabber->start();

	// Main loop.
	while (! viewer->wasStopped())
		// boost::this_thread::sleep(boost::posix_time::seconds(1));

      std::this_thread::sleep_for(1s);


	if (! justVisualize)
		openniGrabber->stop();
}
