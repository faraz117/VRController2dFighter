// ------------------------- OpenPose C++ API Tutorial - Example 16 - Custom Output -------------------------
// Synchronous mode: ideal for production integration. It provides the fastest results with respect to runtime
// performance.
// In this function, the user can implement its own way to render/display/storage the results.

// Third-party dependencies
#include <opencv2/opencv.hpp>
// Command-line user interface
#define OPENPOSE_FLAGS_DISABLE_DISPLAY
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>

#include "keyboard_simulator.hpp"

// Custom OpenPose flags
// Display
DEFINE_bool(no_display, false,
	"Enable to disable the visual display.");

DEFINE_string(write_json, "D:\\CUDA_bin\\2dfighter_build\\x64\\Debug\\json\\", "Directory to write OpenPose output in JSON format. It includes body, hand, and face pose"
	" keypoints (2-D and 3-D), as well as pose candidates (if `--part_candidates` enabled).");
DEFINE_string(write_images, "D:\\CUDA_bin\\2dfighter_build\\x64\\Debug\\img\\", "Directory to write rendered frames in `write_images_format` image format.");

float keypoints_idle[75] = { 429.338,87.7121,0.924312,
430.769,142.044,0.880676,
385.041,143.457,0.827137,
360.749,212.131,0.920498,
365.008,183.489,0.818847,
476.511,141.949,0.819609,
506.543,209.205,0.856954,
466.514,164.813,0.841148,
435.058,282.119,0.585257,
405.014,282.104,0.582984,
383.673,396.497,0.499337,
0.00,0.00,0.00,
463.692,282.106,
0.59781,473.692,
389.33,0.439375,
0.00,0.00,0.00,
417.968,82.005,0.883821,
439.371,79.1299,0.880948,
406.536,89.1688,0.842858,
452.207,87.695,0.901514,
0.00,0.00,0.00,
0.00,0.00,0.00,
0.00,0.00,0.00,
0.00,0.00,0.00,
0.00,0.00,0.00,
0.00,0.00,0.00 };

float keypoints_punch[75]
{ 442.266,86.3447,0.891816,
442.219,141.955,0.865541,
395.02,140.597,0.802594,
330.703,132.069,0.824682,
277.843,100.586,0.818294,
489.392,141.976,0.803202,
519.39,206.37,0.844633,
473.713,156.369,0.863456,
440.852,286.389,0.588886,
409.376,283.549,0.548174,
387.921,392.171,0.52519,
0,0,0,
473.716,286.402,0.576511,
485.066,393.627,0.461765,
0,0,0,
433.653,80.5659,0.836069,
452.265,79.111,0.883972,
420.785,89.1171,0.826612,
465.071,86.3365,0.874732,
0,0,0,
0,0,0,
0,0,0,
0,0,0,
0,0,0,
0,0,0 };

float keypoints_kick[75]{
	576.533,60.5326,0.827134,
	588.081,116.279,0.82235,
	542.311,120.602,0.78465,
	512.305,190.607,0.806281,
	483.672,177.789,0.864355,
	636.672,106.288,0.759266,
	636.643,166.363,0.751387,
	0,0,0,
	549.42,259.25,0.563063,
	519.385,242.076,0.503616,
	396.47,224.952,0.310696,
	259.244,236.4,0.400562,
	580.912,277.821,0.490034,
	578.003,397.897,0.320967,
	0,0,0,
	563.704,50.5328,0.77765,
	587.964,43.4027,0.765904,
	553.743,60.5075,0.264241,
	610.916,46.2227,0.654913,
	0,0,0,
	0,0,0,
	0,0,0,
	227.779,186.328,0.130751,
	0,0,0,
	0,0,0
};


cv::Mat idle_matrix = cv::Mat(1, 75, CV_32F, keypoints_idle);
cv::Mat punch_matrix = cv::Mat(1, 75, CV_32F, keypoints_punch);
cv::Mat kick_matrix = cv::Mat(1, 75, CV_32F, keypoints_kick);


double correlation(cv::Mat &image_1, cv::Mat &image_2) {

	// convert data-type to "float"
	cv::Mat im_float_1;
	image_1.convertTo(im_float_1, CV_32F);
	cv::Mat im_float_2;
	image_2.convertTo(im_float_2, CV_32F);

	int n_pixels = im_float_1.rows * im_float_1.cols;

	// Compute mean and standard deviation of both images
	cv::Scalar im1_Mean, im1_Std, im2_Mean, im2_Std;
	meanStdDev(im_float_1, im1_Mean, im1_Std);
	meanStdDev(im_float_2, im2_Mean, im2_Std);

	// Compute covariance and correlation coefficient
	double covar = (im_float_1 - im1_Mean).dot(im_float_2 - im2_Mean) / n_pixels;
	double correl = covar / (im1_Std[0] * im2_Std[0]);

	return correl;

}

class WKeyboardSimulator : public op::Worker<op::Datum> {

};

// This worker will just read and return all the jpg files in a directory

class WUserOutput : public op::WorkerConsumer<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
{
public:
	void initializationOnThread() {}

	void workConsumer(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
	{
		try
		{
			// User's displaying/saving/other processing here
				// datumPtr->cvOutputData: rendered frame with pose or heatmaps
				// datumPtr->poseKeypoints: Array<float> with the estimated pose
			if (datumsPtr != nullptr && !datumsPtr->empty())
			{
				// Show in command line the resulting pose keypoints for body, face and hands
				op::opLog("\nKeypoints:");
				// Accesing each element of the keypoints
				const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
				op::opLog("Person pose keypoints:");
				for (auto person = 0; person < poseKeypoints.getSize(0); person++)
				{
					auto current_keypoints = poseKeypoints.clone();
					float current_pose[75];
					uint32_t count = 0;
					for (auto bodyPart = 0; bodyPart < poseKeypoints.getSize(1); bodyPart++)
					{
						std::string valueToPrint;
						for (auto xyscore = 0; xyscore < poseKeypoints.getSize(2); xyscore++)
						{
							current_pose[count] = poseKeypoints[{person, bodyPart, xyscore}];
							//valueToPrint += std::to_string(poseKeypoints[{person, bodyPart, xyscore}]) + " ";
							count++;
						}
						//op::opLog(valueToPrint);
					}
					op::opLog("Filled these many values:");
					op::opLog(count);
					cv::Mat current_pose_matrix = cv::Mat(1, 75, CV_32F, current_pose);
					try {
						auto kick_correlation = correlation(current_pose_matrix, kick_matrix);
						auto idle_correlation = correlation(current_pose_matrix, idle_matrix);
						auto punch_correlation = correlation(current_pose_matrix, punch_matrix);
						op::opLog("Kick_correlation");
						op::opLog(kick_correlation);
						op::opLog("Idle_coorelation");
						op::opLog(idle_correlation);
						op::opLog("punch_coorelation");
						op::opLog(punch_correlation);
						if( idle_correlation > 0.8){
							if (kick_correlation > idle_correlation && kick_correlation > punch_correlation) {
								keyboard_simulator.soryogin();
							}
							if (punch_correlation > idle_correlation && punch_correlation > kick_correlation) {
								keyboard_simulator.punch();
							}
						}
					}
					catch (std::exception const& e)
					{
						op::opLog(e.what());
						op::opLog("Kick Matrix Dimensions");
						op::opLog(kick_matrix.rows);
						op::opLog(kick_matrix.cols);
						op::opLog("Current Pose Matrix Dimensions");
						op::opLog(current_pose_matrix.rows);
						op::opLog(current_pose_matrix.cols);
						op::opLog("Template not found");
					}
				}
				op::opLog(" ");                // Display results (if enabled)

				if (!FLAGS_no_display)
				{
					// Display rendered output image
					const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
					cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
					// Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
					const char key = (char)cv::waitKey(1);
					if (key == 27)
						this->stop();
				}
			}
		}
		catch (const std::exception& e)
		{
			this->stop();
			op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
		}
	}
private:
	KeyboardSimulator keyboard_simulator;
};

void configureWrapper(op::Wrapper& opWrapper)
{
	try
	{
		// Configuring OpenPose

		// logging_level
		op::checkBool(
			0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
			__LINE__, __FUNCTION__, __FILE__);
		op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
		op::Profiler::setDefaultX(FLAGS_profile_speed);

		// Applying user defined configuration - GFlags to program variables
		// producerType
		op::ProducerType producerType;
		op::String producerString;
		std::tie(producerType, producerString) = op::flagsToProducer(
			op::String(FLAGS_image_dir), op::String(FLAGS_video), op::String(FLAGS_ip_camera), FLAGS_camera,
			FLAGS_flir_camera, FLAGS_flir_camera_index);
		// cameraSize
		const auto cameraSize = op::flagsToPoint(op::String(FLAGS_camera_resolution), "-1x-1");
		// outputSize
		const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
		// netInputSize
		const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
		// handNetInputSize
		const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
		// poseMode
		const auto poseMode = op::flagsToPoseMode(FLAGS_body);
		// poseModel
		const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
		// JSON saving
		if (!FLAGS_write_keypoint.empty())
			op::opLog(
				"Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json`"
				" instead.", op::Priority::Max);
		// keypointScaleMode
		const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
		// heatmaps to add
		const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
			FLAGS_heatmaps_add_PAFs);
		const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
		// >1 camera view?
		const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
		// Face and hand detectors
		const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
		const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
		// Enabling Google Logging
		const bool enableGoogleLogging = true;

		// Initializing the user custom classes
		// GUI (Display)
		auto wUserOutput = std::make_shared<WUserOutput>();
		// Add custom processing
		const auto workerOutputOnNewThread = true;
		opWrapper.setWorker(op::WorkerType::Output, wUserOutput, workerOutputOnNewThread);
		// Add keyboard simulator
		auto wKeyboardSimulator = std::make_shared<WKeyboardSimulator>();
		opWrapper.setWorker(op::WorkerType::PostProcessing, wKeyboardSimulator,workerOutputOnNewThread);

		// Pose configuration (use WrapperStructPose{} for default and recommended configuration)
		const op::WrapperStructPose wrapperStructPose{
			poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
			FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
			poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
			FLAGS_part_to_show, op::String(FLAGS_model_folder), heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
			(float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max,
			op::String(FLAGS_prototxt_path), op::String(FLAGS_caffemodel_path),
			(float)FLAGS_upsampling_ratio, enableGoogleLogging };
		opWrapper.configure(wrapperStructPose);
		// Hand configuration (use op::WrapperStructHand{} to disable it)
		const op::WrapperStructHand wrapperStructHand{
			FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
			op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
			(float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold };
		opWrapper.configure(wrapperStructHand);
		// Producer (use default to disable any input)
		const op::WrapperStructInput wrapperStructInput{
			producerType, producerString, FLAGS_frame_first, FLAGS_frame_step, FLAGS_frame_last,
			FLAGS_process_real_time, FLAGS_frame_flip, FLAGS_frame_rotate, FLAGS_frames_repeat,
			cameraSize, op::String(FLAGS_camera_parameter_path), FLAGS_frame_undistort, FLAGS_3d_views };
		opWrapper.configure(wrapperStructInput);
		// Output (comment or use default argument to disable any output)
		const op::WrapperStructOutput wrapperStructOutput{
			FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
			op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
			FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
			op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
			op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
			op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
			op::String(FLAGS_udp_port) };
		opWrapper.configure(wrapperStructOutput);
		// No GUI. Equivalent to: opWrapper.configure(op::WrapperStructGui{});
		// Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
		if (FLAGS_disable_multi_thread)
			opWrapper.disableMultiThreading();
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
}

int tutorialApiCpp()
{
	try
	{
		op::opLog("Starting OpenPose demo...", op::Priority::High);
		const auto opTimer = op::getTimerInit();

		// OpenPose wrapper
		op::opLog("Configuring OpenPose...", op::Priority::High);
		op::Wrapper opWrapper;
		configureWrapper(opWrapper);

		// Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
		op::opLog("Starting thread(s)...", op::Priority::High);
		opWrapper.exec();

		// Measuring total time
		op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);

		// Return
		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}
}

int main(int argc, char *argv[])
{
	// Parsing command line flags
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	// Running tutorialApiCpp
	return tutorialApiCpp();
}
