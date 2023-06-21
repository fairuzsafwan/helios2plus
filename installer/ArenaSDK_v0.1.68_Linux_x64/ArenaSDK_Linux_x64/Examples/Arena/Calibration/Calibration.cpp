/***************************************************************************************
 ***                                                                                 ***
 ***  Copyright (c) 2022, Lucid Vision Labs, Inc.                                    ***
 ***                                                                                 ***
 ***  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     ***
 ***  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       ***
 ***  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    ***
 ***  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         ***
 ***  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  ***
 ***  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  ***
 ***  SOFTWARE.                                                                      ***
 ***                                                                                 ***
 ***************************************************************************************/


#include "stdafx.h"
#include "ArenaApi.h"
#include "SaveApi.h"

#define TAB1 "  "
#define TAB2 "    "
#define TAB3 "      "

#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Helios RGB: TritonCalibration
//    This example demonstrates color overlay over 3D image, part 1 -
//    TritonCalibration: Before the data between the two cameras can be combined,
//    we must first calibrate the lens on the Triton color camera to find its
//    optical center and focal length (intrinsics), and lens distortion
//    coefficients (pinhole model). We can achieve this by printing a target with
//    a checkerboard pattern or you can download our calibration target here
//    (15kB, PDF, 8.5 x 11 in)
//    https:arenasdk.s3-us-west-2.amazonaws.com/LUCID_target_whiteCircles.pdf
//    Before calibrating the Triton camera you must focus its lens. Place the
//    target at your application�s working distance and focus the Triton�s
//    lens so that the calibration target is in focus. Calibrating the Triton
//    camera requires grabbing several images of the calibration chart at
//    different positions within the camera�s field of view. At least 3 images
//    are required but 4 to 8 images are typically used to get a better - quality
//    calibration.

// =-=-=-=-=-=-=-=-=-
// =-=- SETTINGS =-=-
// =-=-=-=-=-=-=-=-=-

// image timeout
#define TIMEOUT 200

// number of calibration points to compare
#define NUM_IMAGES 8 //10

// calibration values file name
#define FILE_NAME "/temp/calibration_helios_triton/tritoncalibration.yml"

// orientation values file name
#define FILE_NAME_OUT "/temp/calibration_helios_triton/orientation.yml"

// time to sleep between images (in milliseconds)
#define SLEEP_MS 1000

// =-=-=-=-=-=-=-=-=-
// =-= HELPERS =-=-=-
// =-=-=-=-=-=-=-=-=-

// helper class
class Settings
{
public:
	Settings() :
		goodInput(false)
	{
	}

	enum Pattern
	{
		NOT_EXISTING,
		CHESSBOARD,
		CIRCLES_GRID,
		ASYMMETRIC_CIRCLES_GRID
	};
	enum InputType
	{
		INVALID,
		CAMERA,
		VIDEO_FILE,
		IMAGE_LIST
	};

	cv::Size boardSize; // The size of the board -> Number of items by width and height
	Pattern calibrationPattern; // One of the Chessboard, circles, or asymmetric circle pattern
	float squareSize; // The size of a square in your defined unit (point, millimeter,etc).
	int nrFrames; // The number of frames to use from the input for calibration
	float aspectRatio; // The aspect ratio
	int delay; // In case of a video input
	bool writePoints; // Write detected feature points
	bool writeExtrinsics; // Write extrinsic parameters
	bool calibZeroTangentDist; // Assume zero tangential distortion
	bool calibFixPrincipalPoint; // Fix the principal point at the center
	bool flipVertical; // Flip the captured images around the horizontal axis
	std::string outputFileName; // The name of the file where to write
	bool showUndistorsed; // Show undistorted images after calibration
	std::string input; // The input ->
	bool useFisheye = false; // use fisheye camera model for calibration
	bool fixK1; // fix K1 distortion coefficient
	bool fixK2; // fix K2 distortion coefficient
	bool fixK3; // fix K3 distortion coefficient
	bool fixK4; // fix K4 distortion coefficient
	bool fixK5; // fix K5 distortion coefficient

	int cameraID;
	std::vector<std::string> imageList;
	size_t atImageList;
	cv::VideoCapture inputCapture;
	InputType inputType;
	bool goodInput;
	int flag;

private:
	std::string patternToUse;
};

// helper function
bool findCalibrationPoints(const cv::Mat& image_in_orig, std::vector<cv::Point2f>& grid_centers)
{
	float scaling = 1.0;
	cv::Mat image_in = image_in_orig;

	cv::SimpleBlobDetector::Params bright_params;
	bright_params.filterByColor = true;
	bright_params.blobColor = 255; // white circles in the calibration target
	bright_params.filterByCircularity = true;
	bright_params.minCircularity = 0.8f;

	cv::Ptr<cv::SimpleBlobDetector> blob_detector = cv::SimpleBlobDetector::create(bright_params);

	// pattern_size(num_cols, num_rows) num_cols: number of columns (number of
	// circles in a row) of the calibration target viewed by the camera num_rows:
	// number of rows (number of circles in a column) of the calibration target
	// viewed by the camera Specify according to the orientation of the
	// calibration target
	cv::Size pattern_size(5, 4);

	bool is_found = cv::findCirclesGrid(image_in, pattern_size, grid_centers, 1, blob_detector);

	double scaled_nrows = 2400.0;
	while (!is_found && scaled_nrows >= 100)
	{
		scaled_nrows /= 2.0;
		scaling = static_cast<float>((double)image_in_orig.rows / scaled_nrows);
		cv::resize(image_in_orig, image_in, cv::Size(static_cast<int>((double)image_in_orig.cols / scaling), static_cast<int>((double)image_in_orig.rows / scaling)));

		is_found = cv::findCirclesGrid(image_in, pattern_size, grid_centers, 1, blob_detector);
	}

	// Scale back the grid centers
	for (unsigned int i = 0; i < grid_centers.size(); ++i)
	{
		grid_centers[i].x = grid_centers[i].x * scaling;
		grid_centers[i].y = grid_centers[i].y * scaling;
	}

	return is_found;
}

// helper function
static double computeReprojectionErrors(
	const std::vector<std::vector<cv::Point3f>>& objectPoints,
	const std::vector<std::vector<cv::Point2f>>& imagePoints,
	const std::vector<cv::Mat>& rvecs,
	const std::vector<cv::Mat>& tvecs,
	const cv::Mat& cameraMatrix,
	const cv::Mat& distCoeffs,
	std::vector<float>& perViewErrors,
	bool fisheye)
{
	std::vector<cv::Point2f> imagePoints2;
	size_t totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (size_t i = 0; i < objectPoints.size(); ++i)
	{
		if (fisheye)
		{
			cv::fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix, distCoeffs);
		}
		else
		{
			projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		}
		err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);

		size_t n = objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

// helper function
static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners, Settings::Pattern patternType)
{
	corners.clear();

	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
}

// helper function
static bool calculate(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<std::vector<cv::Point2f>> imagePoints, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs, std::vector<float>& reprojErrs, double& totalAvgErr)
{
	//! [fixed_aspect]
	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	if (s.flag & cv::CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = s.aspectRatio;
	
	//! [fixed_aspect]
	if (s.useFisheye)
	{
		distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
	}
	else
	{
		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
	}

	std::vector<std::vector<cv::Point3f>> objectPoints;
	objectPoints.push_back(std::vector<cv::Point3f>());
	s.boardSize.width = 5;
	s.boardSize.height = 4;
	s.squareSize = 50;

	calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	// find intrinsic and extrinsic camera parameters
	double rms;

	if (s.useFisheye)
	{
		cv::Mat _rvecs, _tvecs;
		rms = cv::fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs, _tvecs, s.flag);

		rvecs.reserve(_rvecs.rows);
		tvecs.reserve(_tvecs.rows);
		for (int i = 0; i < int(objectPoints.size()); i++)
		{
			rvecs.push_back(_rvecs.row(i));
			tvecs.push_back(_tvecs.row(i));
		}
	}
	else
	{
		rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, s.flag);
	}

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs, s.useFisheye);

	return ok;
}

// =-=-=-=-=-=-=-=-=-
// =-=- EXAMPLE -=-=-
// =-=-=-=-=-=-=-=-=-

// calculates and saves calibration values
void CalculateAndSaveCalibrationValues(Arena::IDevice* pDevice, cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
	// get node values that will be changed in order to return their values at
	// the end of the example
	GenICam::gcstring acquisitionModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "AcquisitionMode");
	GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat");

	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(
		pDevice->GetTLStreamNodeMap(),
		"StreamAutoNegotiatePacketSize",
		true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(
		pDevice->GetTLStreamNodeMap(),
		"StreamPacketResendEnable",
		true);

	// Set pixel format
	std::cout << TAB1 << "Set pixel format to 'Mono8'\n";

	Arena::SetNodeValue<GenICam::gcstring>(
		pDevice->GetNodeMap(),
		"PixelFormat",
		"Mono8");

	// set acquisition mode
		std::cout << TAB1 << "Set acquisition mode to 'Continuous'\n";

	Arena::SetNodeValue<GenICam::gcstring>(
		pDevice->GetNodeMap(),
		"AcquisitionMode",
		"Continuous");

	// set buffer handling mode
	std::cout << TAB1 << "Set buffer handling mode to 'NewestOnly'\n";
	Arena::SetNodeValue<GenICam::gcstring>(
		pDevice->GetTLStreamNodeMap(),
		"StreamBufferHandlingMode",
		"NewestOnly");

	// start stream
	pDevice->StartStream();

	// Get sets of calibration points
	std::cout << TAB1 << "Getting " << NUM_IMAGES << " sets of calibration points\n";
	std::cout << TAB1 << "Move the calibration target around the frame for best results\n";
	
	const cv::Size patternSize(5, 4);
	std::vector<std::vector<cv::Point2f>> calibrationPoints;
	cv::Size imageSize;	
	size_t attempts = 0;
	size_t images = 0;
	size_t gridCentersFound = 0;
	size_t successes = 0;

	while (successes < NUM_IMAGES)
	{
		Arena::IImage* pImage = nullptr;
		try
		{
			// get image
			attempts++;
			Arena::IImage* pImage = pDevice->GetImage(TIMEOUT);
			images++;
			if (pImage->IsIncomplete())
				throw std::runtime_error("Incomplete image");

			size_t w = pImage->GetWidth();
			size_t h = pImage->GetHeight();
			imageSize.width = static_cast<int>(w);
			imageSize.height = static_cast<int>(h);

			// Copy data into an OpenCV matrix
			cv::Mat imageMatrix = cv::Mat(imageSize.height, imageSize.width, CV_8UC1);
			memset(imageMatrix.data, 0, w * h);
			memcpy(imageMatrix.data, pImage->GetData(), w * h);

			pDevice->RequeueBuffer(pImage);
			
			// Find calibration circles
			std::vector<cv::Point2f> gridCenters;
			findCalibrationPoints(imageMatrix, gridCenters);
			gridCentersFound = gridCenters.size();

			if (gridCentersFound == 20)
			{
				calibrationPoints.push_back(gridCenters);
				successes++;
			}
		}
		catch (...)
		{
			// on failure, ignore and retry
		}

		std::cout << TAB2 << attempts << " attempts, " << images << " images, " << gridCentersFound << " circles found, " << successes << " calibration points\r";

		// sleep between images
		//std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
		std::getchar();
	};

	// Calculate camera matrix and distance coefficients
	std::cout << "\n" << TAB1 << "Calculate camera matrix and distance coefficients\n";
	
	
	Settings s;
	s.nrFrames = NUM_IMAGES;
	s.inputType = Settings::IMAGE_LIST;
	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool calculationSucceeded = calculate(s, imageSize, cameraMatrix, distCoeffs, calibrationPoints, rvecs, tvecs, reprojErrs, totalAvgErr);
	
	std::cout << TAB2 << "Calibration " << (calculationSucceeded ? "succeeded" : "failed") << "\n";
	std::cout << TAB2 << "Reprojection Error: " << totalAvgErr << "\n";
	
	// Save calibration information
	//std::cout << TAB1 << "Save camera matrix and distance coefficients to file '" << FILE_NAME << "'\n";

	// cv::FileStorage fs(FILE_NAME, cv::FileStorage::WRITE);
	// fs << "cameraMatrix" << cameraMatrix;
	// fs << "distCoeffs" << distCoeffs;
	// fs.release();

	// stop stream
	pDevice->StopStream();

	// return nodes to their initial values
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", pixelFormatInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "AcquisitionMode", acquisitionModeInitial);
}

// =-=-=-=-=-=-=-=-=-
// =- PREPARATION -=-
// =- & CLEAN UP =-=-
// =-=-=-=-=-=-=-=-=-

bool isApplicableDevice(Arena::DeviceInfo deviceInfo)
{
	// color triton camera needed
	return ((deviceInfo.ModelName().find("TRI") != GenICam::gcstring::npos) && (deviceInfo.ModelName().find("-C") != GenICam::gcstring::npos));
}


void getImageHLT(Arena::IDevice* pHeliosDevice, cv::Mat& intensity_image, cv::Mat& xyz_mm)
{
	Arena::SetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "PixelFormat", "Coord3D_ABCY16");

	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(
		pHeliosDevice->GetTLStreamNodeMap(),
		"StreamAutoNegotiatePacketSize",
		true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(
		pHeliosDevice->GetTLStreamNodeMap(),
		"StreamPacketResendEnable",
		true);

	// Read the scale factor and offsets to convert from unsigned 16-bit values 
	//    in the Coord3D_ABCY16 pixel format to coordinates in mm
	GenApi::INodeMap* node_map = pHeliosDevice->GetNodeMap();
	double xyz_scale_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateScale");
	Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateA");
	double x_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");
	Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateB");
	double y_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");
	Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateC");
	double z_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");

	pHeliosDevice->StartStream();
	Arena::IImage* image = pHeliosDevice->GetImage(TIMEOUT);

	size_t height, width;
	height = image->GetHeight();
	width = image->GetWidth();

	xyz_mm = cv::Mat((int)height, (int)width, CV_32FC3);
	intensity_image = cv::Mat((int)height, (int)width, CV_16UC1);

	const uint16_t* input_data;
	input_data = (uint16_t*)image->GetData();

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

			intensity_image.at<ushort>(ir, ic) = input_data[3]; // // Intensity value

			input_data += 4;
		}
	}

	// clean up
	input_data = NULL;
	delete input_data;

	pHeliosDevice->RequeueBuffer(image);
	pHeliosDevice->StopStream();
}



void getImageTRI(Arena::IDevice* pDeviceTriton, cv::Mat& triton_image)
{
	Arena::SetNodeValue<GenICam::gcstring>(pDeviceTriton->GetNodeMap(), "PixelFormat", "RGB8");

	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(
		pDeviceTriton->GetTLStreamNodeMap(),
		"StreamAutoNegotiatePacketSize",
		true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(
		pDeviceTriton->GetTLStreamNodeMap(),
		"StreamPacketResendEnable",
		true);

	pDeviceTriton->StartStream();
	Arena::IImage* pImage = pDeviceTriton->GetImage(TIMEOUT);

	// convert Triton image to mono for dot finding
	Arena::IImage* pConvert = Arena::ImageFactory::Convert(pImage, Mono8);
	size_t w = pImage->GetWidth();
	size_t h = pImage->GetHeight();
	cv::Size patternsize(5, 4); //
	triton_image = cv::Mat((int)h, (int)w, CV_8UC1);
	memcpy(triton_image.data, pConvert->GetData(), h * w);

	// clean up
	Arena::ImageFactory::Destroy(pConvert);
	pDeviceTriton->RequeueBuffer(pImage);
	pDeviceTriton->StopStream();	
}


void findCalibrationPointsHLT(const cv::Mat& image_in, std::vector<cv::Point2f>& grid_centers)
{
	cv::SimpleBlobDetector::Params bright_params;
	bright_params.filterByColor = true;
	bright_params.blobColor = 255; // white circles in the calibration target
	bright_params.thresholdStep = 2;
	bright_params.minArea = 10.0;  // Min/max area can be adjusted based on size of dots in image
	bright_params.maxArea = 1000.0;
	cv::Ptr<cv::SimpleBlobDetector> blob_detector = cv::SimpleBlobDetector::create(bright_params);

	// pattern_size(num_cols, num_rows) num_cols: number of columns (number of
	// circles in a row) of the calibration target viewed by the camera num_rows:
	// number of rows (number of circles in a column) of the calibration target
	// viewed by the camera Specify according to the orientation of the
	// calibration target
	cv::Size pattern_size(5, 4);

	// Find max value in input image
	double min_value, max_value;
	cv::minMaxIdx(image_in, &min_value, &max_value);

	// Scale image to 8-bit, using full 8-bit range
	cv::Mat image_8bit;
	image_in.convertTo(image_8bit, CV_8U, 255.0 / max_value);

	bool is_found = cv::findCirclesGrid(image_8bit, pattern_size, grid_centers, cv::CALIB_CB_SYMMETRIC_GRID, blob_detector);
}


bool findCalibrationPointsTRI(const cv::Mat& image_in_orig, std::vector<cv::Point2f>& grid_centers)
{
	float scaling = 1.0;
	cv::Mat image_in = image_in_orig;

	/*ArenaView*/
	cv::SimpleBlobDetector::Params bright_params;
	bright_params.filterByColor = true;
	bright_params.blobColor = 255; // white circles in the calibration target
	bright_params.filterByCircularity = true;
	bright_params.minCircularity = 0.8f;

	cv::Ptr<cv::SimpleBlobDetector> blob_detector = cv::SimpleBlobDetector::create(bright_params);

	// pattern_size(num_cols, num_rows) num_cols: number of columns (number of
	// circles in a row) of the calibration target viewed by the camera num_rows:
	// number of rows (number of circles in a column) of the calibration target
	// viewed by the camera Specify according to the orientation of the
	// calibration target
	cv::Size pattern_size(5, 4);

	bool is_found = findCirclesGrid(image_in, pattern_size, grid_centers, 1, blob_detector);

	double scaled_nrows = 2400.0;
	while (!is_found && scaled_nrows >= 100)
	{
		scaled_nrows /= 2.0;
		scaling = static_cast<float>((double)image_in_orig.rows / scaled_nrows);
		cv::resize(image_in_orig, image_in, cv::Size(static_cast<int>((double)image_in_orig.cols / scaling), static_cast<int>((double)image_in_orig.rows / scaling)));

		is_found = findCirclesGrid(image_in, pattern_size, grid_centers, 1, blob_detector);
		std::cout << "Found " << grid_centers.size() << "circle centers.\n";
	}

	// Scale back the grid centers
	for (unsigned int i = 0; i < grid_centers.size(); ++i)
	{
		grid_centers[i].x = grid_centers[i].x * scaling;
		grid_centers[i].y = grid_centers[i].y * scaling;
	}
	return is_found;
}


void CalculateAndSaveOrientationValues(Arena::IDevice* pDeviceTRI, Arena::IDevice* pDeviceHLT, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
	// get node values that will be changed in order to return their values at
	// the end of the example
	GenICam::gcstring pixelFormatInitialTRI = Arena::GetNodeValue<GenICam::gcstring>(pDeviceTRI->GetNodeMap(), "PixelFormat");
	GenICam::gcstring pixelFormatInitialHLT = Arena::GetNodeValue<GenICam::gcstring>(pDeviceHLT->GetNodeMap(), "PixelFormat");

	// Read in camera matrix and distance coefficients
	//std::cout << TAB1 << "Read camera matrix and distance coefficients from file '" << FILE_NAME_IN << "'\n";

	// cv::FileStorage fs(FILE_NAME_IN, cv::FileStorage::READ);
	// cv::Mat cameraMatrix;
	// cv::Mat distCoeffs;
	
	// fs["cameraMatrix"] >> cameraMatrix;
	// fs["distCoeffs"] >> distCoeffs;
	
	// fs.release();

	//cameraMatrix = tritonIntMatrix;
	//distCoeffs = tritondistCoeffs;

	// Get an image from Helios 2
	std::cout << TAB1 << "Acquire HLT image\n";

	cv::Mat imageMatrixHLTIntensity;
	cv::Mat imageMatrixHLTXYZ;

	getImageHLT(pDeviceHLT, imageMatrixHLTIntensity, imageMatrixHLTXYZ);

	// Get an image from Triton
	std::cout << TAB1 << "Acquire TRI image\n";

	cv::Mat imageMatrixTRI;

	getImageTRI(pDeviceTRI, imageMatrixTRI);

	// Calculate orientation values
	std::cout << TAB1 << "Calculate orientation values\n";

	std::vector<cv::Point2f> gridCentersHLT;
	std::vector<cv::Point2f> gridCentersTRI;
	
	// find HLT calibration points using HLT intensity image
	std::cout << TAB2 << "Searching points in HLT image\n";

	findCalibrationPointsHLT(imageMatrixHLTIntensity, gridCentersHLT);

	if (gridCentersHLT.size() != 20)
		throw std::logic_error("Unable to find points in HLT intensity image (*there must be 20 points)\n");

	// find TRI calibration points
	std::cout << TAB2 << "Searching points in TRI image\n";
	
	findCalibrationPointsTRI(imageMatrixTRI, gridCentersTRI);
	
	if (gridCentersTRI.size() != 20)
		throw std::logic_error("Unable to find points in TRI image (*there must be 20 points)\n");
	
	// prepare for PnP
	std::cout << TAB2 << "Prepare for PnP\n";
	
	std::vector<cv::Point3f> targetPoints3Dmm;
	std::vector<cv::Point2f> targetPoints3DPixels;
	std::vector<cv::Point2f> targetPoints2DPixels;
	
	for (int i = 0; i < static_cast<int>(gridCentersTRI.size()); i++)
	{
		unsigned int c1 = (unsigned int)round(gridCentersHLT[i].x);
		unsigned int r1 = (unsigned int)round(gridCentersHLT[i].y);
		unsigned int c2 = (unsigned int)round(gridCentersTRI[i].x);
		unsigned int r2 = (unsigned int)round(gridCentersTRI[i].y);

		float x = imageMatrixHLTXYZ.at<cv::Vec3f>(r1, c1)[0];
		float y = imageMatrixHLTXYZ.at<cv::Vec3f>(r1, c1)[1];
		float z = imageMatrixHLTXYZ.at<cv::Vec3f>(r1, c1)[2];

		cv::Point3f pt(x, y, z);
		std::cout << TAB3 << "Point " << i << ": " << pt << "\n";

		targetPoints3Dmm.push_back(pt);
		targetPoints3DPixels.push_back(gridCentersHLT[i]);
		targetPoints2DPixels.push_back(gridCentersTRI[i]);
	}

	cv::Mat rotationVector;
	cv::Mat translationVector;

	bool orientationSucceeded = cv::solvePnP(targetPoints3Dmm,
		targetPoints2DPixels,
		cameraMatrix,
		distCoeffs,
		rotationVector,
		translationVector);

	std::cout << TAB2 << "Orientation " << (orientationSucceeded ? "succeeded" : "failed") << "\n";
	
	// Save orientation information
	std::cout << TAB1 << "Save camera matrix, distance coefficients, and rotation and translation vectors to file '" << FILE_NAME_OUT << "'\n";

	cv::FileStorage fs2(FILE_NAME_OUT, cv::FileStorage::WRITE);

	fs2 << "cameraMatrix" << cameraMatrix;
	fs2 << "distCoeffs" << distCoeffs;
	fs2 << "rotationVector" << rotationVector;
	fs2 << "translationVector" << translationVector;

	fs2.release();
	
	// return nodes to their initial values
	Arena::SetNodeValue<GenICam::gcstring>(pDeviceTRI->GetNodeMap(), "PixelFormat", pixelFormatInitialTRI);
	Arena::SetNodeValue<GenICam::gcstring>(pDeviceHLT->GetNodeMap(), "PixelFormat", pixelFormatInitialHLT);
}


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
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	std::cout << "Helios and Triton Calibration\n";

	try
	{
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

		// Arena::IDevice* pDevice = nullptr;

		// for (auto& deviceInfo : deviceInfos)
		// {
		// 	if (isApplicableDevice(deviceInfo))
		// 	{
		// 		pDevice = pSystem->CreateDevice(deviceInfo);
		// 		break;
		// 	}
		// }

		// if (!pDevice)
		// 	throw std::logic_error("no applicable device");

		// //-------------------------- Step 1 (Grab Triton Images) ------------------------
		// if (pDevice)
		// {
		// 	std::cout << "Commence example\n\n";
		// 	CalculateAndSaveCalibrationValues(pDevice, cameraMatrix, distCoeffs);
		// 	std::cout << "\nExample complete\n";
		// }

	
		Arena::IDevice* pDeviceTRI = nullptr;
		Arena::IDevice* pDeviceHLT = nullptr;
		for (auto& deviceInfo : deviceInfos)
		{
			if (!pDeviceTRI && isApplicableDeviceTriton(deviceInfo))
			{
				pDeviceTRI = pSystem->CreateDevice(deviceInfo);
			}
			else if (isApplicableDeviceTriton(deviceInfo))
			{
				throw std::logic_error("too many Triton devices connected");
			}
			else if (!pDeviceHLT && isApplicableDeviceHelios2(deviceInfo))
			{
				pDeviceHLT = pSystem->CreateDevice(deviceInfo);
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
		

		if (pDeviceTRI && pDeviceHLT)
		{
			std::cout << "Start grabbing images from Triton...\n\n";
			CalculateAndSaveCalibrationValues(pDeviceTRI, cameraMatrix, distCoeffs);
			std::cout << "\nGrabbing images completed!\n\n";

			std::cout << "Start calibrating Helios and Triton\n\n";
			CalculateAndSaveOrientationValues(pDeviceTRI, pDeviceHLT, cameraMatrix, distCoeffs);
			std::cout << "\nCalibration completed!\n";
		}



		// clean up example
		// if (pDevice)
		// 	pSystem->DestroyDevice(pDevice);
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
