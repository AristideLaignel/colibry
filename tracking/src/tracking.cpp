#include "ros/ros.h"
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/io/vpVideoReader.h>
#include <visp/vpVideoWriter.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>

vpVideoWriter writer;
vpMbGenericTracker tracker;
//vpMbEdgeTracker tracker;

std::string opt_videoname_record = "/home/aristide/ros_ws/src/tracking/videos/track1.mp4"; 
bool record = false;
 
int main(int argc, char **argv)
{
  try {
    std::string opt_videoname = "model/track1.mp4";
    std::string opt_modelname = "/home/aristide/ros_ws/src/tracking/model/rectangle.cao";
    int opt_tracker = 1;
 
    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--video")
        opt_videoname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--model")
        opt_modelname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--tracker")
        opt_tracker = atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--record")
        record = true;
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--video <video name>] [--model <model name>]"
                     " [--tracker <0=egde|1=keypoint|2=hybrid>] [--help] [-h]\n"
                  << std::endl;
        return 0;
      }
    }
    std::string parentname = vpIoTools::getParent(opt_modelname);
    std::string objectname = vpIoTools::getNameWE(opt_modelname);
 
    if (!parentname.empty())
      objectname = parentname + "/" + objectname;
 
    std::cout << "Video name: " << opt_videoname << std::endl;
    std::cout << "Tracker requested config files: " << objectname << ".[init, cao]" << std::endl;
    std::cout << "Tracker optional config files: " << objectname << ".[ppm]" << std::endl;
 
    vpImage<vpRGBa> I, I_record;
    vpCameraParameters cam;
    vpHomogeneousMatrix cMo;
 
    vpROSGrabber g;
    g.setMasterURI("http://192.168.12.20:11311/");
    g.setImageTopic("/camera/color/image_raw");
    g.open(I);

    std::cout << "Set Image" << std::endl;
 
    vpDisplay *display = new vpDisplayX();

    std::cout << "Display" << std::endl;

    display->init(I, 100, 100, "Model-based tracker");

 
    // if (opt_tracker == 0 || opt_tracker == 2) {
    //   vpMe me;
    //   me.setMaskSize(5);
    //   me.setMaskNumber(180);
    //   me.setRange(8);
    //   me.setThreshold(10000);
    //   me.setMu1(0.5);
    //   me.setMu2(0.5);
    //   me.setSampleStep(4);
    //   tracker.setMovingEdge(me);
    // }

    tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
 
    //cam.initPersProjWithoutDistortion(1386.7579345703125, 1389.036865234375, 972.1744995117188, 542.078857421875);

    cam.initPersProjWithoutDistortion(616.3367919921875,  622.14404296875,  320.0775146484375,  240.7794189453125);
    tracker.setCameraParameters(cam);
    //tracker.getCameraParameters(cam);

    tracker.setDisplayFeatures(true);
    tracker.loadModel(objectname + ".cao");
    //tracker.setDisplayFeatures(true);
    tracker.setGoodMovingEdgesRatioThreshold(0.5);
    tracker.initClick(I, objectname + ".init", true);

    if(record){
          // Init the writer to record a video
        writer.setCodec(cv::VideoWriter::fourcc('m', 'p', '4', 'v')); // MPEG-1 codec
        writer.setFramerate(1.0);
        writer.setFileName(opt_videoname_record);
        writer.open(I);
    }
 
    while (ros::ok()) {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);
      tracker.track(I);
      tracker.getPose(cMo);
      std::cout << "cMo: " << cMo << std::endl;
      std::cout << "Rotation Vector: " << cMo.getRotationMatrix().getThetaUVector() << std::endl;
      tracker.getCameraParameters(cam);
      tracker.display(I, cMo, cam, vpColor::green, 2);
      vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
      vpDisplay::flush(I);
      vpDisplay::display(I);
      vpDisplay::getImage(I,I_record);
      writer.saveFrame(I_record);
      //vpTime::wait(t,1);
      vpDisplay::flush(I);
      vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);

      
     
      
 
      if (vpDisplay::getClick(I, false))
        break;
    }
    vpDisplay::getClick(I);
    delete display;
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
  }
}

