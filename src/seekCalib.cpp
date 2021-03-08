#include <string>
 
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

int main(int argc, char * argv []) {

    std::cout << "OpenCV version : " << CV_VERSION << std::endl;
    std::cout << "Major version : " << CV_MAJOR_VERSION << std::endl;
    std::cout << "Minor version : " << CV_MINOR_VERSION << std::endl;
    std::cout << "Subminor version : " << CV_SUBMINOR_VERSION << std::endl;

    std::string imageNamePattern("../calibImages/*.jpg");
    std::vector<cv::String> fns;
    cv::glob(imageNamePattern, fns, false);

    std::vector<cv::Mat> calibImages;
    cv::Size imgSize;

    for (auto& fn : fns) {
        cv::Mat img = cv::imread(fn, cv::IMREAD_GRAYSCALE);//read image
        img = ~img;//invert
        calibImages.push_back(img);
        if ((&fn - &fns.front()) == 0) {
            imgSize = img.size();
        }
    }

    int interiorCornersWidth = 8;
    int interiorCornersHeight = 6;

    cv::Size boardSize(interiorCornersWidth,interiorCornersHeight);

    std::vector<std::vector<cv::Point2f>> imagePoints;


    for (auto& img : calibImages) {
        int ind = &img - &calibImages.front();
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(img, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH );

        if(found) {
            std::cout << "Pattern found on image: " << fns[ind] << std::endl;
            cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.1));
            imagePoints.push_back(corners);
        } else {
            std::cout << "No pattern found on image: " << fns[ind] << std::endl;
        }

        if (fns[ind].find("bens")) {
            // drawChessboardCorners(img, boardSize, cv::Mat(corners), found);
            // cv::imshow("Detected Chessboard", img);
            // cv::waitKey();
        }
    }
    
    double squareSideLengthInches = 5.91/6;//measured by hand
    double squareSideLengthM = squareSideLengthInches * 0.0254;

    std::vector<std::vector<cv::Point3f> > objectPoints(1);

    for( int i = 0; i < interiorCornersHeight; ++i) {
        for( int j = 0; j < interiorCornersWidth; ++j ) {
            objectPoints[0].push_back(cv::Point3f(j*squareSideLengthM, i*squareSideLengthM, 0));
        }
    }

    //duplicate to correct length so it matches imagePoints length
    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    //camera Matrix
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    //output vectors
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    std::vector<double> stdDevIntrinsics;
    std::vector<double> stdDevExtrinsics;
    std::vector<double> perViewError;
    double rms = cv::calibrateCamera(objectPoints, imagePoints, imgSize, cameraMatrix,
                            distCoeffs, rvecs, tvecs, stdDevIntrinsics, stdDevExtrinsics, perViewError);    

    std::cout << "RMS: " << rms << std::endl; 

    cv::Mat newCamMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,imgSize,1,imgSize);
    for (auto& img : calibImages) {
        // img = ~img;//un-negate
        int ind = &img - &calibImages.front();
        // cv::drawFrameAxes(img, cameraMatrix,distCoeffs, rvecs[ind], tvecs[ind], .05);


        cv::Mat dst(imgSize, img.depth());
        cv::undistort(img, dst, cameraMatrix,distCoeffs, newCamMatrix);
        cv::imshow("Testing", dst);
        cv::waitKey();
    }
    

}