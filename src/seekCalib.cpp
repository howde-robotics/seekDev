#include <string>
 
#include <opencv2/opencv.hpp>

/*
 * Returns inverted images from a given filepath pattern
 */
std::pair<std::vector<cv::Mat>, std::vector<cv::String>>
getImagesInverted(std::string imageNamePattern) {
    std::vector<cv::String> fns;
    cv::glob(imageNamePattern, fns, false);

    std::vector<cv::Mat> calibImages;

    for (auto& fn : fns) {
        cv::Mat img = cv::imread(fn, cv::IMREAD_GRAYSCALE);//read image
        img = ~img;//invert
        calibImages.push_back(img);
    }
    return {calibImages, fns};
}

/**
 * Finds the interior corner points of a checkerboard calibration plate in a series
 * of images
 */
std::vector<std::vector<cv::Point2f>>
findImagePoints(std::vector<cv::Mat> calibImages, std::vector<cv::String> filenames, cv::Size boardSize) {
    std::vector<std::vector<cv::Point2f>> imagePoints;

    for (auto& img : calibImages) {
        int ind = &img - &calibImages.front();
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(img, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH );

        if(found) {
            std::cout << "Pattern found on image: " << filenames[ind] << std::endl;
            cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.1));
            imagePoints.push_back(corners);
        } else {
            std::cout << "No pattern found on image: " << filenames[ind] << std::endl;
        }

        if (filenames[ind].find("bens")) {
            // drawChessboardCorners(img, boardSize, cv::Mat(corners), found);
            // cv::imshow("Detected Chessboard", img);
            // cv::waitKey();
        }
    }
    return imagePoints;
}

/**
 * Calculates the locations of the interior points of a checkerboard calibration
 * pattern in the frame of the calibration pattern. (i.e. Z=0, all x, y lie along lines
 * and are separated by squareSideLength)
 */
std::vector<std::vector<cv::Point3f> >
determineObjectPoinsCheckerboard(cv::Size boardSize, double squareSideLengthM, int numCalImages) {
    std::vector<std::vector<cv::Point3f> > objectPoints(1);

    for( int i = 0; i < boardSize.height; ++i) {
        for( int j = 0; j < boardSize.width; ++j ) {
            objectPoints[0].push_back(cv::Point3f(j*squareSideLengthM, i*squareSideLengthM, 0));
        }
    }

    //duplicate to correct length so it matches imagePoints length
    objectPoints.resize(numCalImages, objectPoints[0]);
    return objectPoints;
}

int main(int argc, char * argv []) {

    //File path with wildcard to find calibration images
    std::string imageNamePattern("../calibImages/*.jpg");
   
    std::vector<cv::Mat> calibImages;//List of calibration images
    std::vector<cv::String> filenames;//list of calibration image filenames 
    std::tie(calibImages, filenames) = getImagesInverted(imageNamePattern);

    cv::Size imgSize;
    if (calibImages.size() > 0) {
        imgSize = calibImages.front().size();
    }

    int interiorCornersWidth = 8;//number of calibration interior corners width
    int interiorCornersHeight = 6;//number of calibration interior corners height
    cv::Size boardSize(interiorCornersWidth,interiorCornersHeight);

    //container for pixel locations of all interior corners in all images
    std::vector<std::vector<cv::Point2f>> imagePoints = findImagePoints(calibImages, filenames, boardSize);
    
    //real world size of square, enables extrinsics to take proper units
    double squareSideLengthInches = 5.91/6;//measured by hand
    double squareSideLengthM = squareSideLengthInches * 0.0254;

    std::vector<std::vector<cv::Point3f> > objectPoints 
        = determineObjectPoinsCheckerboard(boardSize, squareSideLengthM, imagePoints.size());

    //INTRINSICS containers: camera Matrix and distortion coeffs
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    //EXTRINSICS containter: rotation and translation
    std::vector<cv::Mat> rvecs;//rotation
    std::vector<cv::Mat> tvecs;//translation

    //other containers
    std::vector<double> stdDevIntrinsics;
    std::vector<double> stdDevExtrinsics;
    std::vector<double> perViewError;

    double rms = cv::calibrateCamera(objectPoints, imagePoints, imgSize, cameraMatrix,
                            distCoeffs, rvecs, tvecs, stdDevIntrinsics, stdDevExtrinsics, perViewError);    

    //Matrix to undistort images with the determined intrinsics
    cv::Mat newCamMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,imgSize,1,imgSize);
    for (auto& img : calibImages) {
        // img = ~img;//un-negate
        int ind = &img - &calibImages.front();
        cv::Mat dst;
        cv::cvtColor(img, dst, cv::COLOR_GRAY2BGR);

        cv::drawFrameAxes(dst, cameraMatrix,distCoeffs, rvecs[ind], tvecs[ind], 2 * squareSideLengthM);

        cv::Mat dst2(imgSize, dst.depth());

        cv::undistort(dst, dst2, cameraMatrix,distCoeffs, newCamMatrix);
        cv::imshow("Axis on undistorted Image", dst2);
        cv::waitKey();
    }

    //TODO: Save the calibration parameters to a XML or YAML file
    return 0;
}