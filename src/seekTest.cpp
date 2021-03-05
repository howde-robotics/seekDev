#include <stdio.h>

#include <memory>
#include <vector>
#include <stdexcept>
#include <string>
#include <iostream>

#include <seekware.h>
#include <opencv2/opencv.hpp>

static inline void print_fw_info(psw camera)
{
    sw_retcode status; 
    int therm_ver;

    printf("Model Number:%s\n",camera->modelNumber);
    printf("SerialNumber: %s\n", camera->serialNumber);
    printf("Manufacture Date: %s\n", camera->manufactureDate);

    printf("Firmware Version: %u.%u.%u.%u\n",
            camera->fw_version_major,
			camera->fw_version_minor,
            camera->fw_build_major,
			camera->fw_build_minor);

	status = Seekware_GetSettingEx(camera, SETTING_THERMOGRAPHY_VERSION, &therm_ver, sizeof(therm_ver));
	if (status != SW_RETCODE_NONE) {
		fprintf(stderr, "Error: Seek GetSetting returned %i\n", status);
	}
    printf("Themography Version: %i\n", therm_ver);

	sw_sdk_info sdk_info;
	Seekware_GetSdkInfo(NULL, &sdk_info);
	printf("Image Processing Version: %u.%u.%u.%u\n",
		sdk_info.lib_version_major,
		sdk_info.lib_version_minor,
		sdk_info.lib_build_major,
		sdk_info.lib_build_minor);

	printf("\n");
    fflush(stdout);
}

void freeResources(sw* camera, float* thermData, unsigned short* filteredData) {
	if (thermData != NULL) {
		std::cout << "Freeing therm" << std::endl;
		free(thermData);
	}

	if (filteredData != NULL){
		std::cout << "Freeing filter" << std::endl;
		free(filteredData);
	}
}

void processReturnCode(sw_retcode returnError, std::string context,
	 sw* camera = NULL, float* thermData = NULL, unsigned short* filteredData = NULL) {

	if (returnError != SW_RETCODE_NONE) {
		freeResources(camera, thermData, filteredData);

		std::ostringstream errStringStream;
		errStringStream << "Seek Failed in context: " << context
		 << " \n\tReturn Code: " <<  returnError;
		throw std::runtime_error(errStringStream.str());
	}
	
}

//custom deleter to close camera when unique_ptr goes out of scope
void closeCamera(sw* camera) {
	if (camera != NULL) {
		std::cout << "Closing camera" << std::endl;
		Seekware_Close(camera);
	}
}

int main(int argc, char * argv [])
{
	int numCamsToFind = 1;
	sw_retcode returnError;//error code returned by all seek functions

	sw_sdk_info sdk_info;
	Seekware_GetSdkInfo(NULL, &sdk_info);
	printf("SDK Version: %u.%u\n\n", sdk_info.sdk_version_major, sdk_info.sdk_version_minor);

	std::unique_ptr<sw*[]> cameraList(new sw*[numCamsToFind]);
	int num_cameras_found = 0;

	returnError = Seekware_Find(cameraList.get(), numCamsToFind, &num_cameras_found);
	processReturnCode(returnError, "Finding Camera");
	if (num_cameras_found == 0) {
		std::cerr << "No cameras Available" << std::endl;
		return 1;
	}

	std::unique_ptr<sw, decltype(&closeCamera)> camera(cameraList[0], &closeCamera);

	returnError = Seekware_Open(camera.get());
	processReturnCode(returnError, "Opening Camera", camera.get());

	printf("::Camera Firmware Info::\n");
	print_fw_info(camera.get());

	std::cout << "Camera row, cols: " << camera->frame_rows << ", " << 
		camera->frame_cols << std::endl;

	int totalNumPixels = camera->frame_rows * camera->frame_cols;

	std::vector<float> thermographyData(totalNumPixels);
	std::vector<unsigned short> filteredData(totalNumPixels);

	// Seekware_SetSettingEx(camera.get(), SETTING_ENABLE_TIMESTAMP, &enable, sizeof(enable));
	// Seekware_SetSettingEx(camera.get(), SETTING_RESET_TIMESTAMP, &enable, sizeof(enable));

	return 0;
}