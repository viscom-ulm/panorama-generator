#include <tools.hpp>
#include <util.h>
#include <glm/ext.hpp>
#include <glm/glm.hpp>

#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace immersight;
using namespace viscom;

/**
* @function main
*/
int main(int argn, char **argv)
{

	cv::CommandLineParser parser(argn, argv,
		"{w|2000|}{d|0|}{n||}{@calibration||}"
		"{@filelist||}{@outdir||}");

	std::string calibration = parser.get<std::string>("@calibration");
	std::string filelist = parser.get<std::string>("@filelist");
	std::string outdir = parser.get<std::string>("@outdir");
	int cubemap_pixelsize = parser.get<int>("w");
	int numberOfCameras = parser.get<int>("n");
	bool debug = parser.get<bool>("d");



	if (!parser.check())
	{
		std::cout << "Usage: ./PanoramaGenerator.exe -[w|d|n] calibration.xml images.xml [outputdir]\nw - cube face size in pixel (default 2000)\nd - write debug images (default false)\nn - number of cameras" << std::endl;
		parser.printErrors();
		return -1;
	}

	std::string folderCreateCommand = "mkdir " + outdir;
	system(folderCreateCommand.c_str());

	float imageWidth, imageHeight;
	float nNear = 1.0f;
	float nFar = 20000.0f;
	size_t nCamera;

	std::vector<camera> cameras;
	VecMat CM, D, camera_poses, pattern_poses;
	VecInt pattern_idx;
	VecDouble rms;
	readMulticalib(calibration, CM, D, camera_poses, pattern_poses, pattern_idx, rms);
	glm::vec3 centroid_p = centroid(camera_poses);

	VecStr img_list = readStringList(filelist);
	nCamera = img_list.size();
	if (nCamera != numberOfCameras) return -1;
	VecMat images(nCamera);
	for (size_t i = 0; i < img_list.size(); ++i) {
		cv::Mat image = cv::imread(img_list[i], cv::IMREAD_COLOR);
		if (image.empty()) {
			return -1;
		}
		image.copyTo(images[i]);
		cv::Size imgsize = image.size();
		imageWidth = static_cast<float>(imgsize.width);
		imageHeight = static_cast<float>(imgsize.height);
	}

	VecMat debug_list;
	for (size_t i = 0; i < images.size(); i++)
	{
		debug_list.push_back(images[i].clone());
	}
	float rot_y = 10.0f;
	for (size_t i = 0; i < nCamera; ++i) {
		camera cam;
		if (nCamera == 6) {
			cam.perspective = glm::perspective(glm::radians(90.0f), 1.0f, nNear, nFar);
			switch (CubeFaces(i)) {
			case Right:
				cam.view = glm::lookAt(glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case Left:
				cam.view = glm::lookAt(glm::vec3(0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case Top:
				cam.view = glm::lookAt(glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
				break;
			case Bottom:
				cam.view = glm::lookAt(glm::vec3(0.0f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
				break;
			case Back:
				cam.view = glm::lookAt(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case Front:
				cam.view = glm::lookAt(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			}
			//cam.view = glm::rotate(cam.view,glm::radians(2390.0f),glm::vec3(0,1,1));
		}
		else if (nCamera == 36) {
			cam.perspective = ProjectionMatrixFromCalibratedCamera<float>(CM[i], nNear, nFar, imageWidth, imageHeight);
			// RH to LH
			cam.perspective[1][1] = -cam.perspective[1][1];
			cam.view = glm::translate(ViewMatrixCVtoGL<float>(camera_poses[i]), -centroid_p);
			cam.rms = rms[i];
		}
		cameras.push_back(cam);
	}

	cv::Mat framebuffer;
	cv::Mat T = cv::Mat(cubemap_pixelsize * 3, cubemap_pixelsize * 4, CV_8UC3, cv::Scalar(255, 255, 255));
	for (size_t i = 0; i <6; ++i) {
		framebuffer = cv::Mat::zeros(cubemap_pixelsize, cubemap_pixelsize, CV_8UC3);
		for (int y = 0; y < cubemap_pixelsize; ++y) {
			for (int x = 0; x < cubemap_pixelsize; ++x) {
				glm::vec3 position = get3DpositionFromSkybox(x, y, cubemap_pixelsize, CubeFaces(i));
				cv::Vec3b color = readColorFromCamerasAndInterpolate(images, debug_list, cameras, position, imageWidth, imageHeight, nNear, nFar);
				framebuffer.at<cv::Vec3b>(/*cubemap_pixelsize-1-*/y, /*cubemap_pixelsize-1-*/x) = color;
			}
		}
		std::string file_name = "framebuffer.png";
		switch (CubeFaces(i)) {
		case Right:
			insertMat(cubemap_pixelsize, cubemap_pixelsize * 2, framebuffer, T);
			file_name = "right.png";
			break;
		case Left:
			insertMat(cubemap_pixelsize, 0, framebuffer, T);
			file_name = "left.png";
			break;
		case Top:
			insertMat(cubemap_pixelsize * 2, cubemap_pixelsize, framebuffer, T);
			file_name = "bottom.png";
			break;
		case Bottom:
			insertMat(0, cubemap_pixelsize, framebuffer, T);
			file_name = "top.png";
			break;
		case Back:
			insertMat(cubemap_pixelsize, cubemap_pixelsize * 3, framebuffer, T);
			file_name = "back.png";
			break;
		case Front:
			insertMat(cubemap_pixelsize, cubemap_pixelsize, framebuffer, T);
			file_name = "front.png";
			break;
		default:
			file_name = "framebuffer.png";
			break;
		}
		//cv::imshow(file_name, framebuffer);
		//cv::waitKey(0);
		cv::imwrite(outdir +"/"+ file_name, framebuffer);
	}
	cv::imwrite(outdir+"/T.png", T);
	if (debug) {
		folderCreateCommand = "cd " + outdir + " && mkdir debug";
		system(folderCreateCommand.c_str());
		for (size_t i = 0; i < img_list.size(); ++i) {
			cv::imwrite(outdir + "/debug/" + cameraIndex2string(i) + ".png", debug_list[i]);
		}
	}

	return 0;
}