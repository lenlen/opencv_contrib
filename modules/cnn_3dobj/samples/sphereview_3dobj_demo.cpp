/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <opencv2/cnn_3dobj.hpp>
#include <opencv2/PerlinNoise.hpp>
#include <opencv2/viz/vizcore.hpp>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/PerlinNoise.hpp>

using namespace cv;
using namespace std;
using namespace cv::cnn_3dobj;


int main(int argc, char *argv[]){
	const String keys = "{help | | demo :$ ./sphereview_test -radius=250 -ite_depth=1 -plymodel=../ape.ply -imagedir=../data/images_ape/ -labeldir=../data/label_ape.txt, then press 'q' to run the demo for images generation when you see the gray background and a coordinate.}"
			     "{radius | 250 | Distanse from camera to object, used for adjust view for the reason that differet scale of .ply model.}"
			     "{ite_depth | 1 | Iteration of sphere generation, we add points on the middle of lines of sphere and adjust the radius suit for the original radius.}"
			     "{plymodel | ../ape.ply | path of the '.ply' file for image rendering. }"
			     "{imagedir | ../data/images_ape/ | path of the generated images for one particular .ply model. }"
			     "{labeldir | ../data/label_ape.txt | path of the generated images for one particular .ply model. }";
	cv::CommandLineParser parser(argc, argv, keys);
	parser.about("Demo for Sphere View data generation");
	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}

	Size windowSize( 128, 128 );
	Size resolution( 640, 480 );
	float xratio = (float)windowSize.width/(float)resolution.width;
	float yratio = (float)windowSize.height/(float)resolution.height;
	Matx33d K;
	K.zeros();
  K(0,0) = 518.464*xratio; //fx
  K(1,1) = 518.536*yratio; //fy
  K(0,2) = 310.243*xratio; //cx
  K(1,2) = 231.697*yratio; //cy
  int numLight = 5; //TODO
  int numScale = 3;
  float stepScale = 0.05;

	float radius = parser.get<float>("radius");
	int ite_depth = parser.get<int>("ite_depth");
	string plymodel = parser.get<string>("plymodel");
	string imagedir = parser.get<string>("imagedir");
	string labeldir = parser.get<string>("labeldir");

	vector<cv::cnn_3dobj::IcoSphere> viewSpheres;
	vector<vector<Point3d> > pointsOnSpheres;
	for(int i = 0; i < numScale; i++){
	  cv::cnn_3dobj::IcoSphere viewSphere(radius+(i*stepScale),ite_depth);
	  viewSpheres.push_back(viewSphere);
	  vector<Point3d> campos = viewSphere.CameraPos;
	  pointsOnSpheres.push_back(campos);
	}

	std::fstream imglabel;
	char* p=(char*)labeldir.data();
	imglabel.open(p);

	bool camera_pov = true;
	/// Create a window
	viz::Viz3d myWindow("Coordinate Frame");
	/// Add coordinate axes
	//myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
	myWindow.setBackgroundColor(viz::Color::black());



	viz::Mesh objmesh = viz::Mesh::loadOBJ(plymodel);
	//viz::Mesh objmesh = viz::Mesh::load(plymodel);
  Mat t = imread("/home/len/src/Logicos/logicos-utils/datasetGenerator/desktop/data/misuratore_open/model_texture_flip.jpg");
  objmesh.texture = t;

  viz::WMesh mesh_widget(objmesh);

  cv::viz::Camera cam(K, windowSize);
  // cv::viz::Camera cam = myWindow.getCamera();
  //cam.setFov(Vec2d(1.1057, 0.86494));
  myWindow.setCamera(cam);
  //myWindow.addNoise();

  cv::cnn_3dobj::PerlinNoise perlin_noise;

	/// Set background color
	/// Let's assume camera has the following properties

  for(size_t pose = 0; pose < pointsOnSpheres.at(0).size(); pose++){
    for(int currentScale = 0; currentScale < numScale; currentScale++){
	    for(int currentLight = 0; currentLight < numLight; currentLight++){
        //TODO rest only for test
        Point3d currentPoint = pointsOnSpheres.at(currentScale).at(pose);

        cv::Mat perlin_noise_img = perlin_noise.CreatePerlinNoiseImage(windowSize);
        //cv::imshow("perlin_noise_img", perlin_noise_img);
        //waitKey(0);

        //myWindow.setBackgroundTexture(perlin_noise_img);
        imglabel << currentPoint.x << ' ' << currentPoint.y << ' ' << currentPoint.z << endl;
        /// We can get the pose of the cam using makeCameraPoses
        Affine3f cam_pose = viz::makeCameraPose(currentPoint, Point3d(0,0,0), Point3d(0,-1,0));
        /// Create a cloud widget.

        /// Visualize camera frame
        /*if (!camera_pov)
        {
          viz::WCameraPosition cpw(1); // Coordinate axes
          viz::WCameraPosition cpw_frustum(Vec2f(0.5, 0.5)); // Camera frustum
          myWindow.showWidget("CPW", cpw, cam_pose);
          myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
        }*/

        /// Visualize widget
        //mesh_widget.setRenderingProperty(viz::LINE_WIDTH, 4.0);
        //myWindow.addRandomLight();
        myWindow.showWidget("ape", mesh_widget);//, cloud_pose_global);

        /*viz::WLine axis(cam_focal_point, campos->at(pose)*23);
        axis.setRenderingProperty(viz::LINE_WIDTH, 4.0);
        myWindow.showWidget("Line Widget", axis);*/

        /// Set the viewer pose to that of camera
        if (camera_pov)
          myWindow.setViewerPose(cam_pose);
        char temp[100];
        sprintf (temp,"s%d_p%d",currentScale, pose);
        string filename = temp;
        filename = imagedir + filename;
        filename += ".png";
        //myWindow.saveScreenshot(filename);
        myWindow.spin();
	    }
	  }
	}
	return 1;
};
