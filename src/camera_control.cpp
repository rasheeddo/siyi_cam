#include <memory>
#include <chrono>
#include <stdio.h>
#include <cassert>
#include <iostream>
#include <type_traits>
#include <math.h>
#include <libusb-1.0/libusb.h>
#include <libuvc/libuvc.h>
#include <assert.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <cstdint>

#define VID_WIDTH  1280
#define VID_HEIGHT 720
#define VIDEO_OUT "/dev/video30"

#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <strings.h>  // for bzero()

#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

#define SIYI_IP "192.168.144.25"
#define SIYI_PORT 37260
#define BUF_LEN 512

using namespace std::chrono_literals;

struct uvc_context;
typedef struct uvc_context uvc_context_t;
struct uvc_device;
typedef struct uvc_device uvc_device_t;
struct uvc_device_handle;
typedef struct uvc_device_handle uvc_device_handle_t;
struct uvc_frame;
typedef struct uvc_frame uvc_frame_t;

// std::string rtsp_url = "rtsp://192.168.1.59:8554/main.264";
// std::string gst_string = "gst-launch-1.0 rtspsrc location=rtsp://192.168.1.59:8554/main.264 ! rtph265depay ! avdec_h265 ! videoconvert ! videoscale ! video/x-raw,width=1280,height=720,format=BGR ! appsink sync=false";
std::string gst_string;
std::string rtsp_url;
static cv::VideoCapture CAP;

const uint16_t table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t to_big_endian(uint16_t value) {
    uint16_t result = 0;
    result |= (value & 0x00FF) << 8;
    result |= (value & 0xFF00) >> 8;
    return result;
}

void attach_crc(uint8_t* data_array, int length, uint8_t* result){

    uint16_t crc = 0;
    for (int i=0; i<length; i++){
        crc = ((crc << 8)&0xFF00) ^ table[((crc >> 8)&0xFF) ^ data_array[i]];
        crc &= 0xFFFF;
    }

    uint16_t crc_big = to_big_endian(crc);

    uint8_t crc_1st = (uint8_t)((crc_big >> 8) & 0xFF);
    uint8_t crc_2nd = (uint8_t)(crc_big & 0xFF);

    for (int k=0; k<(length+2); k++){
        if (k < length){
            result[k] = data_array[k];
        } else if (k == length) {
            result[k] = crc_1st;
        } else {
            result[k] = crc_2nd;
        }

        // std::cout << std::hex << static_cast<int>(result[k]) << std::endl; 
    }
}

class SIYI_Cam : public rclcpp::Node {

    public: 

        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "success";
            
            for (const auto &parameter : parameters){
                
                if ((parameter.get_name() == "cam_ip") && (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)){
                    this->cam_ip = parameter.as_string();
                    RCLCPP_INFO(this->get_logger(), "cam_ip: %s", this->cam_ip.c_str());
                } else if ((parameter.get_name() == "video_out") && (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)){
                    this->video_out = parameter.as_string();
                    RCLCPP_INFO(this->get_logger(), "video_out: %s", this->video_out.c_str());
                } else if ((parameter.get_name() == "use_gst") && (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)){
                    this->use_gst = parameter.as_bool();
                    RCLCPP_INFO(this->get_logger(), "use_gst: %d", this->use_gst);
                } 
            }
            return result;
        }

        SIYI_Cam()  : Node("siyi_cam_node"){
            RCLCPP_INFO(this->get_logger(), "start siyi_cam");

            ////////////////////////
            /// ROS params setup ///
            ////////////////////////
            RCLCPP_INFO(this->get_logger(), "Using parameters as below");
            this->declare_parameter("cam_ip", SIYI_IP);
            this->declare_parameter("video_out", VIDEO_OUT);
            this->declare_parameter("use_gst", true);
			
			this->cam_ip = this->get_parameter("cam_ip").as_string();
            this->video_out = this->get_parameter("video_out").as_string();
            this->use_gst = this->get_parameter("use_gst").as_bool();

            RCLCPP_INFO(this->get_logger(), "cam_ip: %s", this->cam_ip.c_str());
            RCLCPP_INFO(this->get_logger(), "video_out: %s", this->video_out.c_str());
            RCLCPP_INFO(this->get_logger(), "use_gst: %d", this->use_gst);

			param_callback_handle_ = this->add_on_set_parameters_callback(
				std::bind(&SIYI_Cam::parametersCallback, this, std::placeholders::_1));

            /////////////////
            /// UPD setup ///
            /////////////////
            memset((char*)&server, 0, sizeof(server));
            server.sin_family = AF_INET;
            server.sin_port = htons(SIYI_PORT);
            server.sin_addr.s_addr = inet_addr(this->cam_ip.c_str());

            sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock < 0) {
                RCLCPP_ERROR(this->get_logger(), "Error opening socket");
                exit(1);
                //return EXIT_FAILURE;
            }

            ////////////////////
            /// Setup camera ///
            ////////////////////
            
            // gst_string = std::format("gst-launch-1.0 rtspsrc location=rtsp://%s:8554/main.264 ! rtph265depay ! avdec_h265 ! videoconvert ! videoscale ! video/x-raw,width=1280,height=720,format=BGR ! appsink sync=false", 
            //     this->cam_ip);
            // rtsp_url = std::format("rtsp://%s:8554/main.264",
            //     this->cam_ip)

            std::stringstream ss;
            ss << "gst-launch-1.0 rtspsrc location=rtsp://" << this->cam_ip.c_str() << ":8554/main.264 ! rtph265depay ! avdec_h265 ! videoconvert ! videoscale ! video/x-raw,width=1280,height=720,format=BGR ! appsink sync=false";
            gst_string = ss.str();

            std::stringstream sss;
            sss << "rtsp://" << this->cam_ip.c_str() << ":8554/main.264";
            rtsp_url = sss.str();
            
            this->init_cv();
            this->initVideoOut();
            this->init_data_packets();
            sendto(sock, _follow_mode, 11, 0, (const struct sockaddr*)&server, sizeof(server));

            // Pub/Sub //
            rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data; //rmw_qos_profile_default; //rmw_qos_profile_sensor_data; //rmw_qos_profile_best_available
	        image_pub = image_transport::create_publisher(this, "/siyi/image", custom_qos);
            gimbal_sub = this->create_subscription<std_msgs::msg::Int8MultiArray>
			("/siyi/gimbal", 10, std::bind(&SIYI_Cam::gimbal_callback, this, std::placeholders::_1));
            zoom_sub = this->create_subscription<std_msgs::msg::Int8MultiArray>
			("/siyi/zoom", 10, std::bind(&SIYI_Cam::zoom_callback, this, std::placeholders::_1));
            mode_sub = this->create_subscription<std_msgs::msg::Int8>
			("/siyi/mode", 10, std::bind(&SIYI_Cam::mode_callback, this, std::placeholders::_1));

            /// Loop ///
			timer_ = this->create_wall_timer(33ms, std::bind(&SIYI_Cam::timer_callback, this));
        }

    private:

        ////////////
        /// Init ///
        ////////////
        void init_cv(){
            if (this->use_gst){
                CAP.open(gst_string, cv::CAP_GSTREAMER);
            } else {
                CAP.open(rtsp_url);
            }
        }

        void initVideoOut() {
			this->output_video = open(this->video_out.c_str(), O_RDWR);
			if(this->output_video < 0) {
			    RCLCPP_ERROR(this->get_logger(), "video out: could not open output VIDEO_OUT!\n");
			    this->allow_video_out = false;
			} else {
				this->allow_video_out = true;
			}

			// acquire video format from device
			struct v4l2_format vid_format;
			memset(&vid_format, 0, sizeof(vid_format));
			vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

			if (ioctl(this->output_video, VIDIOC_G_FMT, &vid_format) < 0) {
				RCLCPP_ERROR(this->get_logger(), "video out: unable to get video format!\n");
				this->allow_video_out = false;
				exit(1);
			} else {
				this->allow_video_out = true;
			}
			

			// configure desired video format on device
			this->framesize = VID_WIDTH * VID_HEIGHT * 3;
			vid_format.fmt.pix.width = VID_WIDTH;
			vid_format.fmt.pix.height = VID_HEIGHT;
			vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
			vid_format.fmt.pix.sizeimage = this->framesize;
			vid_format.fmt.pix.field = V4L2_FIELD_NONE;

			if (ioctl(this->output_video, VIDIOC_S_FMT, &vid_format) < 0) {
				RCLCPP_ERROR(this->get_logger(), "video out: unable to set video format!\n");
				this->allow_video_out = false;
				exit(1);
			} else {
				this->allow_video_out = true;
			}

		}

        void init_data_packets(){

            attach_crc(zoom_in, 9, _zoom_in);
            attach_crc(zoom_out, 9, _zoom_out);
            attach_crc(zoom_stop, 9, _zoom_stop);
            attach_crc(yaw_r, 10, _yaw_r);
            attach_crc(yaw_l, 10, _yaw_l);
            attach_crc(pitch_up, 10, _pitch_up);
            attach_crc(pitch_down, 10, _pitch_down);
            attach_crc(stop, 10, _stop);
            attach_crc(center, 9, _center);
            attach_crc(follow_mode, 9, _follow_mode);
            attach_crc(lock_mode, 9, _lock_mode);
            attach_crc(fpv_mode, 9, _fpv_mode);

        }

        /////////////////////
        /// ROS callbacks ///
        /////////////////////
        void gimbal_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg){

            int8_t left = msg->data[0];
            int8_t up = msg->data[1];
            int8_t center = msg->data[2];
            int8_t down = msg->data[3];
            int8_t right = msg->data[4];
			RCLCPP_INFO(this->get_logger(), "Got gimbal left: %d up: %d center: %d down: %d right: %d", 
                left, up, center, down, right);

            if ((left == 0) && (up == 0) && (center == 0) && (down == 0) && (right == 0)){
                sendto(sock, _stop, 12, 0, (const struct sockaddr*)&server, sizeof(server));
            } else {
                if (left == 1){
                    sendto(sock, _yaw_l, 12, 0, (const struct sockaddr*)&server, sizeof(server));
                } else if (up == 1){
                    sendto(sock, _pitch_up, 12, 0, (const struct sockaddr*)&server, sizeof(server));
                } else if (center == 1){
                    sendto(sock, _center, 11, 0, (const struct sockaddr*)&server, sizeof(server));
                } else if (down == 1){
                    sendto(sock, _pitch_down, 12, 0, (const struct sockaddr*)&server, sizeof(server));
                } else if (right == 1){
                    sendto(sock, _yaw_r, 12, 0, (const struct sockaddr*)&server, sizeof(server));
                }
            }
			
		}

        void zoom_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
            int8_t zoomIn = msg->data[0];
            int8_t zoomOut = msg->data[1];
            RCLCPP_INFO(this->get_logger(), "Got zoom in: %d out: %d", 
                zoomIn, zoomOut);

            if ((zoomIn == 0) && (zoomOut == 0)){
                sendto(sock, _zoom_stop, 11, 0, (const struct sockaddr*)&server, sizeof(server));
            } else {
                if (zoomIn == 1){
                    sendto(sock, _zoom_in, 11, 0, (const struct sockaddr*)&server, sizeof(server)); 
                } else if (zoomOut == 1){
                    sendto(sock, _zoom_out, 11, 0, (const struct sockaddr*)&server, sizeof(server)); 
                }
            }
        }

        void mode_callback(const std_msgs::msg::Int8::SharedPtr msg) {

            int8_t mode = msg->data;
        
            RCLCPP_INFO(this->get_logger(), "Got mode: %d ", mode);

            if (mode == 0){
                sendto(sock, _follow_mode, 11, 0, (const struct sockaddr*)&server, sizeof(server)); 
            } else if (mode == 1){
                sendto(sock, _fpv_mode, 11, 0, (const struct sockaddr*)&server, sizeof(server)); 
            } else if (mode == 2){
                sendto(sock, _lock_mode, 11, 0, (const struct sockaddr*)&server, sizeof(server)); 
            }

        }

        ////////////
        /// Loop ///
        ////////////
        void timer_callback(){

            if (!CAP.isOpened()) {
                std::cerr << "Error opening RTSP stream" << std::endl;
                got_frame = false;
            } else {
                got_frame = true;
            }

            
            if (got_frame){

                CAP.read(this->frame);

                int row = this->frame.rows;
                int cols = this->frame.cols;
                int ch = this->frame.channels();

                if ((row == 720) && (cols == 1280)){
                    // cv::imshow("frame", this->frame);
                    std_msgs::msg::Header hdr;
                    sensor_msgs::msg::Image::SharedPtr image_msg;
                    image_msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg(); //bgr8
                    image_pub.publish(image_msg);

                    if (this->allow_video_out){
                        cv::Mat result;
                        cv::Mat frame_cvt;
                        cv::cvtColor(frame, frame_cvt, cv::COLOR_BGR2RGB);
    
                        result = frame_cvt.clone();
                        size_t written = write(this->output_video, result.data, this->framesize);
                        if (written < 0) {
                            RCLCPP_ERROR(this->get_logger(), "ERROR: could not write to output device!\n");
    
                        }
                        // std::cout << result.size << std::endl;
                    }
                    
                }
                // RCLCPP_INFO(this->get_logger(), "rows: %d col: %d ch: %d \n", row, cols, ch);
                // cv::waitKey(1);
            }
            
        }

        /// ROS params ///
        std::string cam_ip = SIYI_IP;
        std::string video_out = VIDEO_OUT;
        bool use_gst = true;
           
        /// Video ////
        cv::Mat frame;
        bool got_frame = false;

        int output_video;
		bool allow_video_out;
		size_t framesize;

        /// Socket ///
        struct sockaddr_in server;
        int sock;

        /// UDP packets ///
        uint8_t zoom_in[9]     = {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x05, 0x01};
        uint8_t zoom_out[9]    = {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x05, 0xFF};
        uint8_t zoom_stop[9]   = {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x05, 0x00};
        uint8_t yaw_r[10]      = {0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, 0x07, 0x16, 0x00};
        uint8_t yaw_l[10]      = {0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, 0x07, 0xEA, 0x00};
        uint8_t pitch_up[10]   = {0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, 0x07, 0x00, 0x16};
        uint8_t pitch_down[10] = {0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, 0x07, 0x00, 0xEA};
        uint8_t stop[10]       = {0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00};
        uint8_t center[9]      = {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x01};
        uint8_t follow_mode[9] = {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0C, 0x04};
        uint8_t lock_mode[9]   = {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0C, 0x03};
        uint8_t fpv_mode[9]    = {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0C, 0x05};

        uint8_t _zoom_in[11];
        uint8_t _zoom_out[11];
        uint8_t _zoom_stop[11];
        uint8_t _yaw_r[12];
        uint8_t _yaw_l[12];
        uint8_t _pitch_up[12];
        uint8_t _pitch_down[12];
        uint8_t _stop[12];
        uint8_t _center[11];
        uint8_t _follow_mode[11];
        uint8_t _lock_mode[11];
        uint8_t _fpv_mode[11];


        /// ROS Pub/Sub ///
		image_transport::Publisher image_pub;
        rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr gimbal_sub;
        rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr zoom_sub;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub;

        rclcpp::TimerBase::SharedPtr timer_;

        /// ROS parameters callback ///
		OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[]){

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SIYI_Cam>());
	rclcpp::shutdown();

	return 0;
}