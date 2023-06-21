
// Calibrate RGBD: Converts depth map from point cloud and calibrates RGB and Depth map

#include "stdafx.h"
#include "ArenaApi.h"
#include "SaveApi.h"

#define TAB1 "  "
#define TAB2 "    "

// ROS
#include "boost/multi_array.hpp"

#include <iostream>
#include <fstream>
#include <sstream> //std::stringstream

#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



// =-=-=-=-=-=-=-=-=-
// =-=- SETTINGS =-=-
// =-=-=-=-=-=-=-=-=-

// image timeout
#define TIMEOUT 200

// orientation values file name
#define FILE_NAME_IN "/temp/calibration_helios_triton/orientation.yml"

// file name
#define FILE_NAME_OUT "/temp/calibration_helios_triton/original_"

// Calibrated Depth map of size 2048 x 1536
#define FILE_NAME_DEPTHMAP_16UC1_CALIBRATED "/temp/calibration_helios_triton/depth_16UC1_calibrated.png"

// Calibrated Depth map of size 2048 x 1536
#define FILE_NAME_RGB_CALIBRATED "/temp/calibration_helios_triton/rgb_calibrated.png"

// Calibrated Depth map of size 2048 x 1536
#define FILE_NAME_RGB_CALIBRATED_TIFF "/temp/calibration_helios_triton/rgb_calibrated.tiff"

// =-=-=-=-=-=-=-=-=-=-
// =-=- Functions -=-=-
// =-=-=-=-=-=-=-=-=-=-

void getImageHLT(Arena::IDevice* pHeliosDevice, Arena::IImage** ppOutImage, cv::Mat& xyz_mm, size_t& width, size_t& height, double& xyz_scale_mm, double& x_offset_mm, double& y_offset_mm, double& z_offset_mm)
{
	// Read the scale factor and offsets to convert from unsigned 16-bit values 
	//    in the Coord3D_ABCY16 pixel format to coordinates in mm
	GenApi::INodeMap* node_map = pHeliosDevice->GetNodeMap();
	xyz_scale_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateScale");
	Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateA");
	x_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");
	Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateB");
	y_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");
	Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateC");
	z_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");

	pHeliosDevice->StartStream();
	Arena::IImage* pHeliosImage = pHeliosDevice->GetImage(2000);

	// copy image because original will be delited after function call
	Arena::IImage* pCopyImage = Arena::ImageFactory::Copy(pHeliosImage);
	*ppOutImage = pCopyImage;

	width = pHeliosImage->GetWidth();
	height = pHeliosImage->GetHeight();

	xyz_mm = cv::Mat((int)height, (int)width, CV_32FC3);

	const uint16_t* input_data = reinterpret_cast<const uint16_t*>(pHeliosImage->GetData());
	for (unsigned int ir = 0; ir < height; ++ir)
	{
		for (unsigned int ic = 0; ic < width; ++ic)
		{
			// Get unsigned 16 bit values for X,Y,Z coordinates
			ushort x_u16 = input_data[0];
			ushort y_u16 = input_data[1];
			ushort z_u16 = input_data[2];

			// Convert 16-bit X,Y,Z to float values in mm
			xyz_mm.at<cv::Vec3f>(ir, ic)[0] = (float)(x_u16 * xyz_scale_mm + x_offset_mm);
			xyz_mm.at<cv::Vec3f>(ir, ic)[1] = (float)(y_u16 * xyz_scale_mm + y_offset_mm);
			xyz_mm.at<cv::Vec3f>(ir, ic)[2] = (float)(z_u16 * xyz_scale_mm + z_offset_mm);

			input_data += 4;
		}
	}
	pHeliosDevice->RequeueBuffer(pHeliosImage);
	pHeliosDevice->StopStream();
}


void getImageTRI(Arena::IDevice* pDeviceTriton, Arena::IImage** ppOutImage, cv::Mat& triton_rgb, size_t& width_TRI, size_t& height_TRI)
{
	#if defined(_WIN32)
		Arena::SetNodeValue<GenICam::gcstring>(pDeviceTriton->GetNodeMap(), "PixelFormat", "RGB8");
	#elif defined(__linux__)
		Arena::SetNodeValue<GenICam::gcstring>(pDeviceTriton->GetNodeMap(), "PixelFormat", "BGR8");
	#endif

	pDeviceTriton->StartStream();
	Arena::IImage* pImage = pDeviceTriton->GetImage(2000);

	// copy image because original will be delited after function call
	Arena::IImage* pCopyImage = Arena::ImageFactory::Copy(pImage);
	*ppOutImage = pCopyImage;

	//size_t triHeight, triWidth;
	height_TRI = pImage->GetHeight();
	width_TRI = pImage->GetWidth();
	triton_rgb = cv::Mat((int)height_TRI, (int)width_TRI, CV_8UC3);
	memcpy(triton_rgb.data, pImage->GetData(), height_TRI * width_TRI * 3);

	pCopyImage = NULL;
	delete pCopyImage;

	pDeviceTriton->RequeueBuffer(pImage);
	pDeviceTriton->StopStream();
}

void calibrateDepthRGB(Arena::IDevice* pDeviceTRI, Arena::IDevice* pDeviceHLT)
{
	// get node values that will be changed in order to return their values at
	// the end of the example
	GenICam::gcstring pixelFormatInitialTRI = Arena::GetNodeValue<GenICam::gcstring>(pDeviceTRI->GetNodeMap(), "PixelFormat");
	GenICam::gcstring pixelFormatInitialHLT = Arena::GetNodeValue<GenICam::gcstring>(pDeviceHLT->GetNodeMap(), "PixelFormat");

	// Read in camera matrix, distance coefficients, and rotation and translation vectors
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Mat rotationVector;
	cv::Mat translationVector;

	std::cout << "Reading Camera Parameters in " << FILE_NAME_IN << std::endl;

	cv::FileStorage fs(FILE_NAME_IN, cv::FileStorage::READ);
	
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;
	fs["rotationVector"] >> rotationVector;
	fs["translationVector"] >> translationVector;

	fs.release();

	// Get an image from Helios 2
	std::cout << TAB1 << "Acquire Helios(Raw Depth) image\n";

	Arena::IImage* pImageHLT = nullptr;
	cv::Mat imageMatrixXYZ;
	size_t width = 0;
	size_t height = 0;
	double scale;
	double offsetX, offsetY, offsetZ;

	getImageHLT(
		pDeviceHLT, 
		&pImageHLT, 
		imageMatrixXYZ, 
		width, 
		height, 
		scale, 
		offsetX, 
		offsetY, 
		offsetZ);

	cv::imwrite(FILE_NAME_OUT "XYZ.jpg", imageMatrixXYZ);

	// Get an image from Triton
	std::cout << TAB1 << "Acquire Triton(RGB) image\n";

	Arena::IImage* pImageTRI = nullptr;
	cv::Mat imageMatrixRGB;

	size_t width_TRI = 0;
	size_t height_TRI = 0;

	getImageTRI(
		pDeviceTRI,
		&pImageTRI,
		imageMatrixRGB,
		width_TRI,
		height_TRI);

	cv::imwrite(FILE_NAME_OUT "RGB.jpg", imageMatrixRGB);
	
	int size = imageMatrixXYZ.rows * imageMatrixXYZ.cols;
	cv::Mat xyzPoints = imageMatrixXYZ.reshape(3, size);
	
	// project points
	std::cout << TAB2 << "Project points\n";

	cv::Mat projectedPointsTRI;
	
	cv::projectPoints(
		xyzPoints, 
		rotationVector, 
		translationVector, 
		cameraMatrix, 
		distCoeffs, 
		projectedPointsTRI);

	

	// loop through projected points to access RGB data at those points
	std::cout << TAB2 << "Acquire depth values at projected points\n";

	uint8_t* pColorData = new uint8_t[width * height * 3];

	//Initialize calibrated RGB data
	std::vector<uchar> rgbData(width_TRI * height_TRI * 3);

	//Resize calibrated RGB data to 2048 x 1536 x 3 channels
	//rgbData.resize(width_TRI * height_TRI * 3);

	// float focal_X = cameraMatrix.at<double>(0,0);
	// float focal_Y = cameraMatrix.at<double>(1,1);
	// float center_X = cameraMatrix.at<double>(0,2);
	// float center_Y = cameraMatrix.at<double>(1,2);

	// Create a Mat object for claibrated depth map image with size 1536 x 2048 
	cv::Mat depth_calib((int)height_TRI, (int)width_TRI, CV_32FC1, cv::Scalar(0));
	cv::Mat depth_calib_tiff((int)height_TRI, (int)width_TRI, CV_32FC1, cv::Scalar(0));

	cv::Mat rgb_calib;

	//Undistort image
	cv::undistort(imageMatrixRGB, rgb_calib, cameraMatrix, distCoeffs);
	
	//cv::Mat projectedUndistortedPoints;
	//cv::undistortPoints(projectedPointsTRI, projectedUndistortedPoints, cameraMatrix, distCoeffs);

	for (int i = 0; i < width * height; i++)
	{
		unsigned int colTRI = (unsigned int)std::round(projectedPointsTRI.at<cv::Vec2f>(i)[0]); //projectedPointsTRI
		unsigned int rowTRI = (unsigned int)std::round(projectedPointsTRI.at<cv::Vec2f>(i)[1]); //projectedPointsTRI

		// only handle appropriate points
		if (rowTRI < 0 ||
			colTRI < 0 ||
			rowTRI >= static_cast<unsigned int>(imageMatrixRGB.rows) ||
			colTRI >= static_cast<unsigned int>(imageMatrixRGB.cols))
			continue;

		// access corresponding XYZ and RGB data
		uchar R = rgb_calib.at<cv::Vec3b>(rowTRI, colTRI)[0]; //imageMatrixRGB or rgb_calib
		uchar G = rgb_calib.at<cv::Vec3b>(rowTRI, colTRI)[1]; //imageMatrixRGB or rgb_calib
		uchar B = rgb_calib.at<cv::Vec3b>(rowTRI, colTRI)[2]; //imageMatrixRGB or rgb_calib
		
		float X = imageMatrixXYZ.at<cv::Vec3f>(i)[0];
		float Y = imageMatrixXYZ.at<cv::Vec3f>(i)[1];
		float Z = imageMatrixXYZ.at<cv::Vec3f>(i)[2];

		//Fill Z data into calibrated (x,y) 2D coordinates <-- at this point the projected 2D coordinates is expanded to fit the 2048x1536 size.
		depth_calib.at<float>(rowTRI, colTRI) = Z;

		// grab RGB data to save colored .ply
		pColorData[i * 3 + 0] = B;
		pColorData[i * 3 + 1] = G;
		pColorData[i * 3 + 2] = R;
	}

	//get new camera matrix
	//cv::Mat newCamMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(height_TRI, width_TRI), 1, cv::Size(height_TRI, width_TRI));
	depth_calib_tiff = depth_calib.clone();

	//convert //CV_32FC1 to CV_16UC1
	depth_calib.convertTo(depth_calib, CV_16UC1);

	//cv::Mat rgb_calib((int)height_TRI, (int)width_TRI, CV_8UC3, rgbData.data());

	//sensor_msgs::CameraInfo info = cam_info;
	//img_depth_pub_ = new ros::Publisher(nh_.advertise<sensor_msgs::Image>("calibrated_depth", 1));

	//sensor_msgs::ImagePtr outputDepthImage = cv_bridge::CvImage(std_msgs::Header(), "16UC1", depth_calib).toImageMsg();
	//outputDepthImage->header = info.header;
	//outputDepthImage->header.stamp = info.header.stamp;
	//publish_result.publish(outputDepthImage);
	//publish_cam_info.publish(info);

	//img_depth_pub_->publish(*outputDepthImage);

	//save calibrated RGB image
	cv::imwrite(FILE_NAME_RGB_CALIBRATED, rgb_calib);

	//save calibrated Depthimage 
	cv::imwrite(FILE_NAME_DEPTHMAP_16UC1_CALIBRATED, depth_calib);

	//save calibrated Depthimage 
	cv::imwrite(FILE_NAME_RGB_CALIBRATED_TIFF, depth_calib_tiff);

	// Save original RGB from Triton
	std::cout << TAB1 << "Save uncalibrated RGB image to " << FILE_NAME_OUT << "\n";
	// Save calibrated RGB
	std::cout << TAB1 << "Save calibrated RGB image to " << FILE_NAME_RGB_CALIBRATED << "\n";
	// Save calibrated Depth Map
	std::cout << TAB1 << "Save calibrated Depth Map image to " << FILE_NAME_DEPTHMAP_16UC1_CALIBRATED << "\n";
	// Save calibrated Depth Map
	std::cout << TAB1 << "Save calibrated Depth Map image to " << FILE_NAME_RGB_CALIBRATED_TIFF << "\n";
	

	// prepare to save
	Save::ImageParams params(
		pImageHLT->GetWidth(),
		pImageHLT->GetHeight(),
		pImageHLT->GetBitsPerPixel());

	Save::ImageWriter plyWriter(
		params,
		FILE_NAME_OUT);

	// save .ply with color data
	bool filterPoints = true;
	bool isSignedPixelFormat = false;

	plyWriter.SetPly(
		".ply", 
		filterPoints, 
		isSignedPixelFormat, 
		scale, 
		offsetX, 
		offsetY, 
		offsetZ);

	plyWriter.Save(pImageHLT->GetData(), pColorData);
	
	// return nodes to their initial values
	Arena::SetNodeValue<GenICam::gcstring>(pDeviceTRI->GetNodeMap(), "PixelFormat", pixelFormatInitialTRI);
	Arena::SetNodeValue<GenICam::gcstring>(pDeviceHLT->GetNodeMap(), "PixelFormat", pixelFormatInitialHLT);

	// clean up
	pColorData = NULL;
	delete[] pColorData;
	Arena::ImageFactory::Destroy(pImageHLT);
	Arena::ImageFactory::Destroy(pImageTRI);
}

// =-=-=-=-=-=-=-=-=-
// =- PREPARATION -=-
// =- & CLEAN UP =-=-
// =-=-=-=-=-=-=-=-=-

bool isApplicableDeviceTriton(Arena::DeviceInfo deviceInfo)
{
	// color triton camera needed
	return ((deviceInfo.ModelName().find("TRI") != GenICam::gcstring::npos) && (deviceInfo.ModelName().find("-C") != GenICam::gcstring::npos));
}

bool isApplicableDeviceHelios2(Arena::DeviceInfo deviceInfo)
{
	return ((deviceInfo.ModelName().find("HLT") != GenICam::gcstring::npos) || (deviceInfo.ModelName().find("HTP") != GenICam::gcstring::npos) \
		|| (deviceInfo.ModelName().find("HTW") != GenICam::gcstring::npos));
}

int main()
{
	// flag to track when an exception has been thrown
	bool exceptionThrown = false;

	std::cout << "---- Node to convert Point cloud to depth map and calibrate RGB ---- \n";

	try
	{
		std::ifstream ifile;
		ifile.open(FILE_NAME_IN);
		if (!ifile)
		{
			std::cout << "File '" << FILE_NAME_IN << "' not found\nPlease run examples 'Cpp_HLTRGB_1_Calibration' and 'Cpp_HLTRGB_2_Orientation' prior to this one\nPress enter to complete\n";
			std::getchar();
			return 0;
		}

		// prepare example
		Arena::ISystem* pSystem = Arena::OpenSystem();
		pSystem->UpdateDevices(100);
		std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
		if (deviceInfos.size() == 0)
		{
			std::cout << "\nNo camera connected\nPress enter to complete\n";
			std::getchar();
			return 0;
		}

		Arena::IDevice* pDeviceTRI = nullptr;
		Arena::IDevice* pDeviceHLT = nullptr;
		for (auto& deviceInfo : deviceInfos)
		{
			if (!pDeviceTRI && isApplicableDeviceTriton(deviceInfo))
			{
				pDeviceTRI = pSystem->CreateDevice(deviceInfo);

				// enable stream auto negotiate packet size
				Arena::SetNodeValue<bool>(
					pDeviceTRI->GetTLStreamNodeMap(),
					"StreamAutoNegotiatePacketSize",
					true);

				// enable stream packet resend
				Arena::SetNodeValue<bool>(
					pDeviceTRI->GetTLStreamNodeMap(),
					"StreamPacketResendEnable",
					true);
			}
			else if (isApplicableDeviceTriton(deviceInfo))
			{
				throw std::logic_error("too many Triton devices connected");
			}
			else if (!pDeviceHLT && isApplicableDeviceHelios2(deviceInfo))
			{
				pDeviceHLT = pSystem->CreateDevice(deviceInfo);

				// enable stream auto negotiate packet size
				Arena::SetNodeValue<bool>(
					pDeviceHLT->GetTLStreamNodeMap(),
					"StreamAutoNegotiatePacketSize",
					true);

				// enable stream packet resend
				Arena::SetNodeValue<bool>(
					pDeviceHLT->GetTLStreamNodeMap(),
					"StreamPacketResendEnable",
					true);
			}
			else if (isApplicableDeviceHelios2(deviceInfo))
			{
				throw std::logic_error("too many Helios 2 devices connected");
			}
		}

		if (!pDeviceTRI)
			throw std::logic_error("No applicable Triton devices");

		if (!pDeviceHLT)
			throw std::logic_error("No applicable Helios 2 devices");

		// run example
		if (pDeviceTRI && pDeviceHLT)
		{
			std::cout << "Acquiring depth map and calibrating RGB\n\n";
			calibrateDepthRGB(pDeviceTRI, pDeviceHLT);
			std::cout << "\n----- Completed ----\n";
		}

		if (pDeviceTRI)
			pSystem->DestroyDevice(pDeviceTRI);
		if (pDeviceHLT)
			pSystem->DestroyDevice(pDeviceHLT);

		Arena::CloseSystem(pSystem);
	}
	catch (GenICam::GenericException& ge)
	{
		std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
		exceptionThrown = true;
	}
	catch (std::exception& ex)
	{
		std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
		exceptionThrown = true;
	}
	catch (...)
	{
		std::cout << "\nUnexpected exception thrown\n";
		exceptionThrown = true;
	}

	std::cout << "Press enter to complete\n";
	std::getchar();

	if (exceptionThrown)
		return -1;
	else
		return 0;
}
