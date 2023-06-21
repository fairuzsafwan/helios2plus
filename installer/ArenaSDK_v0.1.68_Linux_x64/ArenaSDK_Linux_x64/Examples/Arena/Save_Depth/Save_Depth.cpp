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
#include <chrono>

#define TAB1 "  "
#define TAB2 "    "
#define TAB3 "      "

// Helios: Min/Max Depth
//    This example captures a 3D image and interprets the ABCY data into their
//    appropriate x, y and z coordinates and intensities. It converts this data
//    into millimeters and then displays this data for points with both the
//    largest and smallest values of z.

// =-=-=-=-=-=-=-=-=-
// =-=- SETTINGS =-=-
// =-=-=-=-=-=-=-=-=-

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// file name
#define FILE_NAME "/temp/output/Helios_MinMaxDepth.ply"
#define FILE_NAME_DEPTHMAP_16UC1 "/temp/output/depthmap_16UC1.png"
#define FILE_NAME_DEPTHMAP_8UC1 "/temp/output/depthmap_8UC1.png"
#define FILE_NAME_DEPTHMAP_16UC1_2 "/temp/output/depthmap2_16UC1.png"
#define FILE_NAME_DEPTHMAP_8UC1_2 "/temp/output/depthmap2_8UC1.png"

// pixel format
//#define PIXEL_FORMAT "Coord3D_ABCY16"
#define PIXEL_FORMAT "Coord3D_ABC16"
#define PIXEL_FORMAT_BMP "Mono16"

// image timeout
#define IMAGE_TIMEOUT 2000

// =-=-=-=-=-=-=-=-=-
// =-=- EXAMPLE -=-=-
// =-=-=-=-=-=-=-=-=-

// store x, y, z data in mm and intensity for a given point
struct PointData
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t intensity;
};

// demonstrates acquiring 3D data for a specific point
// (1) gets image
// (2) interprets ABCY data to get x, y, z and intensity
// (3) stores data for point with min and max z values
// (4) displays 3D data for min and max points
void AcquireImageAndInterpretData(Arena::IDevice* pDevice, size_t k)
{
	GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();

	// validate if Scan3dCoordinateSelector node exists. If not - probaly not
	// Helios camera used running the example
	GenApi::CEnumerationPtr checkpCoordSelector = pNodeMap->GetNode("Scan3dCoordinateSelector");
	if (!checkpCoordSelector)
	{
		std::cout << TAB1 << "Scan3dCoordinateSelector node is not found. Please make sure that Helios device is used for the example.\n";
		return;
	}

	// validate if Scan3dCoordinateOffset node exists. If not - probaly Helios
	// has an old firmware
	GenApi::CFloatPtr checkpCoord = pNodeMap->GetNode("Scan3dCoordinateOffset");
	if (!checkpCoord)
	{
		std::cout << TAB1 << "Scan3dCoordinateOffset node is not found. Please update Helios firmware.\n";
		return;
	}

	// check if Helios2 camera used for the example
	bool isHelios2 = false;
	GenICam::gcstring deviceModelName = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName");
	std::string deviceModelName_tmp = deviceModelName.c_str();
	if (deviceModelName_tmp.rfind("HLT", 0) == 0 || deviceModelName_tmp.rfind("HTP", 0) == 0)
	{
		isHelios2 = true;
	}


	// get node values that will be changed in order to return their values at
	// the end of the example
	GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat");
	GenICam::gcstring operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode");

	//Set Working Distance Mode Distance3000mmSingleFreq
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", "Distance1250mmSingleFreq");

    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dHDRMode", "StandardHDR");



	// Set pixel format
	//    Warning: HLT003S-001 / Helios2 - has only Coord3D_ABCY16 in this case
	//    This example demonstrates data interpretation for both a signed or
	//    unsigned pixel format. Default PIXEL_FORMAT here is set to
	//    Coord3D_ABCY16 but this can be modified to be a signed pixel format by
	//    changing it to Coord3D_ABCY16s.
	std::cout << TAB1 << "Set " << PIXEL_FORMAT << " to pixel format\n";

	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", PIXEL_FORMAT);

	// set operating mode distance
	if (isHelios2)
	{
		std::cout << TAB1 << "Set 3D operating mode to Distance3000mm\n";
		Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", "Distance3000mmSingleFreq");
	}
	else
	{
		std::cout << TAB1 << "Set 3D operating mode to Distance1500mm\n";
		Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", "Distance1500mm");
	}

	// get the coordinate scale in order to convert x, y and z values to mm as
	// well as the offset for x and y to correctly adjust values when in an
	// unsigned pixel format
	std::cout << TAB1 << "Get xyz coordinate scales and offsets\n\n";

	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateA");
	// getting scaleX as float by casting since SetPly() will expect it passed as
	// float
	float scaleX = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale"));
	// getting offsetX as float by casting since SetPly() will expect it passed
	// as float
	float offsetX = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateOffset"));
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateB");
	double scaleY = Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale");
	// getting offsetY as float by casting since SetPly() will expect it passed
	// as float
	float offsetY = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateOffset"));
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateC");
	double scaleZ = Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale");

	//Instrinsic Camera Parameters (K)
	auto focal_X = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibFocalLengthX");
	auto focal_Y = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibFocalLengthY");
	auto center_X = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibOpticalCenterX");
	auto center_Y = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibOpticalCenterY");
	auto distort_val = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibLensDistortionValue");


	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

	// retrieve image
	std::cout << TAB2 << "Acquire image\n";

	pDevice->StartStream();
	Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);

	// prepare info from input buffer
	size_t width = pImage->GetWidth();
	size_t height = pImage->GetHeight();
	size_t size = width * height;
	size_t srcBpp = pImage->GetBitsPerPixel();
	size_t srcPixelSize = srcBpp / 8;
	const uint8_t* pInput = pImage->GetData();
	const uint8_t* pIn = pInput;

	// prepare memory output buffer
	size_t dstBpp = 16;//Arena::GetBitsPerPixel(PIXEL_FORMAT_BMP);
	size_t dstPixelSize = dstBpp / 8;				  // divide by the number of bits in a byte
	size_t dstDataSize = width * height * dstBpp / 8; // divide by the number of bits in a byte
	uint16_t* pOutput = new uint16_t[dstDataSize];
	memset(pOutput, 0, dstDataSize);
	uint16_t* pOut = pOutput;

	// minDepth z value is set to 32767 to guarantee closer points exist as this
	// is the largest value possible
	PointData minDepth = { 0, 0, 32767, 0 };
	PointData maxDepth = { 0, 0, 0, 0 };

	// find points with min and max z values
	std::cout << TAB2 << "Find points with min and max z values\n";

	// using strcmp to avoid conversion issue
	int compareResult_ABCY16s = strcmp(PIXEL_FORMAT, "Coord3D_ABCY16s"); // if they are equal compareResult_ABCY16s = 0
	//int compareResult_ABCY16 = strcmp(PIXEL_FORMAT, "Coord3D_ABCY16");	 // if they are equal compareResult_ABCY16 = 0
	int compareResult_ABC16 = strcmp(PIXEL_FORMAT, "Coord3D_ABC16");	 // if they are equal compareResult_ABCY16 = 0

	bool isSignedPixelFormat = false;

	cv::Mat cv_image = cv::Mat(height, width, CV_32FC1,cv::Scalar(std::numeric_limits<float>::max()));
	int countA = 0;
	int countB = 0;

	float img_array[size] = {};



	// if PIXEL_FORMAT is equal to Coord3D_ABCY16s
	if (compareResult_ABCY16s == 0)
	{
		isSignedPixelFormat = true;

		for (size_t i = 0; i < size; i++)
		{
			// Extract point data to signed 16 bit integer
			//    The first channel is the x coordinate, second channel is the y
			//    coordinate, the third channel is the z coordinate and the
			//    fourth channel is intensity. We offset pIn by 2 for each
			//    channel because pIn is an 8 bit integer and we want to read it
			//    as a 16 bit integer.
			int16_t x = *reinterpret_cast<const int16_t*>(pIn);
			int16_t y = *reinterpret_cast<const int16_t*>((pIn + 2));
			int16_t z = *reinterpret_cast<const int16_t*>((pIn + 4));
			//int16_t intensity = *reinterpret_cast<const int16_t*>((pIn + 6));

			// convert x, y and z values to mm using their coordinate scales
			x = int16_t(double(x) * scaleX);
			y = int16_t(double(y) * scaleY);
			z = int16_t(double(z) * scaleZ);

			if (z < minDepth.z && z > 0)
			{
				minDepth.x = x;
				minDepth.y = y;
				minDepth.z = z;
				//minDepth.intensity = intensity;
			}
			else if (z > maxDepth.z)
			{
				maxDepth.x = x;
				maxDepth.y = y;
				maxDepth.z = z;
				//maxDepth.intensity = intensity;
			}

			pIn += srcPixelSize;
		}
	}
	// if PIXEL_FORMAT is equal to Coord3D_ABCY16
	else if (compareResult_ABC16 == 0)
	{
		size_t pixel_x = 0;
		size_t pixel_y = 0;
		
		for (size_t i = 0; i < size; i++)
		{
			// Extract point data to signed 16 bit integer
			//    The first channel is the x coordinate, second channel is the y
			//    coordinate, the third channel is the z coordinate and the
			//    fourth channel is intensity. We offset pIn by 2 for each
			//    channel because pIn is an 8 bit integer and we want to read it
			//    as a 16 bit integer.
			uint16_t x = *reinterpret_cast<const uint16_t*>(pIn);
			uint16_t y = *reinterpret_cast<const uint16_t*>((pIn + 2));
			uint16_t z = *reinterpret_cast<const uint16_t*>((pIn + 4));

			// if z is less than max value, as invalid values get filtered to
			// 65535
			if (z < 65535)
			{
				// Convert x, y and z to millimeters
				//    Using each coordinates' appropriate scales, convert x, y
				//    and z values to mm. For the x and y coordinates in an
				//    unsigned pixel format, we must then add the offset to our
				//    converted values in order to get the correct position in
				//    millimeters.

				x = uint16_t((double(x) * scaleX) + offsetX);
				y = uint16_t((double(y) * scaleY) + offsetY);
				//x = uint16_t((double(x) * scaleX) + offsetX + offset_x_temp);
				//y = uint16_t((double(y) * scaleY) + offsetY + offset_y_temp);
				z = uint16_t(double(z) * scaleZ);

				int u = static_cast<int>(std::round(focal_X * x / z + center_X));
        		int v = static_cast<int>(std::round(focal_Y * y / z + center_Y));

				if (u >= 0 && u < width && v >= 0 && v < height) {
					float& depth = cv_image.at<float>(v, u);
					if (depth == 0 || depth > z) {
						depth = z;
					}
				}
				img_array[i] = z;
			}
			*pOut = static_cast<int16_t>(z);
			pIn += srcPixelSize;
			pOut += dstPixelSize;
		}
	}
	else
	{
		std::cout << "This example requires the camera to be in either 3D image format Coord3D_ABCY16 or Coord3D_ABCY16s\n\n";
	}

	std::cout << "Helios Internal Camera Parameters: " << std::endl;
	std::cout << "focal_X: " << focal_X << std::endl;
	std::cout << "focal_Y: " << focal_Y << std::endl;
	std::cout << "center_X: " << center_X << std::endl;
	std::cout << "center_Y: " << center_Y << std::endl;
	
	std::cout << "Writing Files...." << std::endl;
	
	//Create mat object of data type 32FC1
	cv::Mat cv_image2 = cv::Mat(height, width, CV_32FC1, img_array);
	cv::Mat cv_image2_16UC1 = cv_image2.clone();
	cv::Mat cv_image2_8UC1 = cv_image2.clone();

	//Convert from 32FC1 to 16UC1
	cv_image2_16UC1.convertTo(cv_image2_16UC1, CV_16UC1);
	cv::imwrite(FILE_NAME_DEPTHMAP_16UC1_2, cv_image2_16UC1);

	cv_image2_8UC1.convertTo(cv_image2_8UC1, CV_8UC1);
	cv::imwrite(FILE_NAME_DEPTHMAP_8UC1_2, cv_image2_8UC1);
	
	cv_image.convertTo(cv_image,CV_16UC1);
	cv::imwrite(FILE_NAME_DEPTHMAP_16UC1, cv_image); 
	cv_image.convertTo(cv_image,CV_8UC1);
	cv::imwrite(FILE_NAME_DEPTHMAP_8UC1, cv_image); 
	//cv::imwrite("/temp/output/depthmap_"+std::to_string(k)+"_8UC1.png", cv_image);
	std::cout << "Writing Files successfull!" << std::endl;

	// prepare image parameters and writer
	Save::ImageParams params(
		pImage->GetWidth(),
		pImage->GetHeight(),
		pImage->GetBitsPerPixel());

	Save::ImageWriter writer(
		params,
		FILE_NAME);

	// set parameters for SetPly()
	bool filterPoints = true; // default
	float offsetZ = 0.0f;	  // default

	// set the output file format of the image writer to .ply
	writer.SetPly(".ply",
				filterPoints,
				isSignedPixelFormat,
				scaleX, // using scaleX as scale since all scales = 0.25f
				offsetX,
				offsetY,
				offsetZ);

	// save image
	writer << pImage->GetData();

	std::cout << TAB2 << "Save image to " << writer.GetLastFileName() << "\n\n";

	// clean up
	pInput = NULL;
	pOutput = NULL;
	delete[] pInput;
	delete[] pOutput;
	pIn = NULL;
	pOut = NULL;
	delete[] pIn;
	delete[] pOut;
	pDevice->RequeueBuffer(pImage);
	pDevice->StopStream();

	// return nodes to their initial values
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", operatingModeInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", pixelFormatInitial);
	//std::cout << TAB1 << "Nodes were set back to initial values\n";
}

// =-=-=-=-=-=-=-=-=-
// =- PREPARATION -=-
// =- & CLEAN UP =-=-
// =-=-=-=-=-=-=-=-=-

int main()
{
	// flag to track when an exception has been thrown
	bool exceptionThrown = false;

	std::cout << "Cpp_Helios_MinMaxDepth\n";

	// Start the timer
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	
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
		Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfos[0]);

		//std::cout << "Commence example\n\n";

		size_t loop_num = 1;

		for (size_t k = 0; k < loop_num; k++)
		{
			// run example
			AcquireImageAndInterpretData(pDevice, k);
		}
		// End the timer
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

		// Calculate the duration
		std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);

		// Print the computation time
		std::cout << "Computation time: " << duration.count() << " seconds" << std::endl;	
		//std::cout << "\nExample complete\n";

		// clean up example
		pSystem->DestroyDevice(pDevice);
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
