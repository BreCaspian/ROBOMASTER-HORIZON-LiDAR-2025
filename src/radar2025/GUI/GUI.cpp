#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <radar2025/Location.h>
#include <radar2025/Locations.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <map>
#include <deque>

using namespace std;
using namespace cv;

const Scalar THEME_COLOR(44, 112, 198);
const Scalar ACCENT_COLOR(255, 140, 0);      
const Scalar TEXT_COLOR(220, 220, 220);      
const Scalar BG_COLOR(30, 30, 30);           
const Scalar PANEL_COLOR(50, 50, 50);        
const Scalar TEAM_RED(0, 0, 255);            
const Scalar TEAM_BLUE(255, 0, 0);           
const Scalar STATUS_GREEN(0, 255, 0);        
const Scalar STATUS_RED(0, 0, 255);          
const Scalar SWITCH_ON_COLOR(0, 153, 255);   
const Scalar SWITCH_OFF_COLOR(80, 80, 80);

static cv_bridge::CvImageConstPtr result_image;
static vector<radar2025::Location> locs;
static float fps = 0.0f;
static int frame_count = 0;
static auto last_time = std::chrono::high_resolution_clock::now();
static bool toggle_states[6] = {false, true, true, false, false, false}; 
static bool toggle_clicked = false;
static int clicked_toggle_id = -1;
static bool uart_test_enabled = false;
static int uart_test_interval = 100;
static bool show_uart_test_info = false; 
static bool bytetracker_enabled = false; 
static ros::NodeHandle* g_nh = nullptr;

const int MAX_TRAJECTORY_LENGTH = 20;
const int TRAJECTORY_FADE_FRAMES = 60;       
typedef std::deque<Point2f> Trajectory;      
static std::map<int, Trajectory> trajectories; 
static int trajectory_fade_counter = 0;

void locations_msgCallback(const radar2025::Locations::ConstPtr &msg)
{
    static std::vector<radar2025::Location> prev_locs;
    locs.clear();
    for (auto &it : msg->locations)
    {
        locs.emplace_back(it);
        if (bytetracker_enabled) {
            if (it.x == 0 && it.y == 0)
                continue;
            Trajectory& traj = trajectories[it.id];
            traj.push_back(Point2f(it.y, it.x));
            if (traj.size() > MAX_TRAJECTORY_LENGTH) {
                traj.pop_front();
            }
            trajectory_fade_counter = TRAJECTORY_FADE_FRAMES;
        }
    }
    if (trajectory_fade_counter > 0) {
        trajectory_fade_counter--;
    } else if (!bytetracker_enabled) {
        trajectories.clear();
    }
    prev_locs = locs;
}
void image_msgCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        result_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        frame_count++;
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_diff = std::chrono::duration<float>(current_time - last_time).count();
        if (time_diff >= 1.0f) {
            fps = frame_count / time_diff;
            frame_count = 0;
            last_time = current_time;
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void drawRoundedRect(Mat& img, Rect rect, Scalar color, int radius, int thickness = 1, bool fill = false)
{
    int type = fill ? -1 : thickness;

    rectangle(img, Rect(rect.x + radius, rect.y, rect.width - 2*radius, rect.height), color, type);
    rectangle(img, Rect(rect.x, rect.y + radius, rect.width, rect.height - 2*radius), color, type);

    circle(img, Point(rect.x + radius, rect.y + radius), radius, color, type);
    circle(img, Point(rect.x + rect.width - radius, rect.y + radius), radius, color, type);
    circle(img, Point(rect.x + radius, rect.y + rect.height - radius), radius, color, type);
    circle(img, Point(rect.x + rect.width - radius, rect.y + rect.height - radius), radius, color, type);
}

void drawShadowText(Mat& img, const string& text, Point position, int fontFace,
                   double fontScale, Scalar textColor, int thickness = 1, int shadowOffset = 1)
{
    putText(img, text, Point(position.x + shadowOffset, position.y + shadowOffset),
            fontFace, fontScale, Scalar(0, 0, 0, 150), thickness);

    putText(img, text, position, fontFace, fontScale, textColor, thickness);
}

void drawStatusIndicator(Mat& img, Point center, bool status, int radius = 6)
{
    circle(img, center, radius + 2, BG_COLOR, -1);
    circle(img, center, radius, status ? STATUS_GREEN : STATUS_RED, -1);

    circle(img, Point(center.x - radius/3, center.y - radius/3), radius/3,
           Scalar(255, 255, 255, 180), -1);
}

void drawToggleButton(Mat& img, Rect btnRect, const string& label, bool state, int id, bool& clicked)
{
    if (clicked && id == clicked_toggle_id) {
        clicked = false;
        if (id >= 0 && id < 6) {
            toggle_states[id] = !toggle_states[id];
        }
    }

    Scalar bgColor = state ? SWITCH_ON_COLOR : SWITCH_OFF_COLOR;
    drawRoundedRect(img, btnRect, bgColor, 15, 1, true);

    int circleRadius = btnRect.height / 2 - 4;
    Point circleCenter;
    if (state) {
        circleCenter = Point(btnRect.x + btnRect.width - circleRadius - 4, 
                           btnRect.y + btnRect.height/2);
    } else {
        circleCenter = Point(btnRect.x + circleRadius + 4, 
                           btnRect.y + btnRect.height/2);
    }

    circle(img, circleCenter, circleRadius, Scalar(255, 255, 255), -1);
    circle(img, Point(circleCenter.x - circleRadius/3, circleCenter.y - circleRadius/3),
          circleRadius/3, Scalar(255, 255, 255, 200), -1);

    drawShadowText(img, label, Point(btnRect.x + btnRect.width + 10, btnRect.y + btnRect.height/2 + 5),
                  FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);
}

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN) {
        vector<Rect>* toggleRects = (vector<Rect>*)userdata;
        if (!toggleRects) return;

        for (size_t i = 0; i < toggleRects->size(); i++) {
            if (x >= (*toggleRects)[i].x && x <= (*toggleRects)[i].x + (*toggleRects)[i].width &&
                y >= (*toggleRects)[i].y && y <= (*toggleRects)[i].y + (*toggleRects)[i].height) {

                toggle_clicked = true;
                clicked_toggle_id = static_cast<int>(i);
                ROS_INFO("Button clicked: index=%d, toggleRects size=%zu", clicked_toggle_id, toggleRects->size());
                if (clicked_toggle_id < 6) {
                    if (clicked_toggle_id == 4) {
                        uart_test_enabled = !uart_test_enabled;
                        toggle_clicked = false;
                        ROS_INFO("UART Test %s", uart_test_enabled ? "enabled" : "disabled");
                        show_uart_test_info = uart_test_enabled;

                        if (g_nh) {
                            g_nh->setParam("/radar2025/UARTTestEnabled", uart_test_enabled);
                            if (uart_test_enabled) {
                                if (g_nh->hasParam("/radar2025/uart_test/interval")) {
                                    g_nh->getParam("/radar2025/uart_test/interval", uart_test_interval);
                                    g_nh->setParam("/radar2025/UARTTestInterval", uart_test_interval);
                                }
                            }
                        }
                    }
                    else if (clicked_toggle_id == 5) {
                        bytetracker_enabled = !bytetracker_enabled;
                        toggle_clicked = false;
                        ROS_INFO("ByteTracker %s", bytetracker_enabled ? "enabled" : "disabled");

                        if (g_nh) {
                            g_nh->setParam("/gui/UseByteTracker", bytetracker_enabled);
                        }
                    }
                    else {
                    }
                }
                break;
            }
        }
    }
}

void drawControlPanel(Mat& img, int x, int y, int width, int height,
                     vector<Rect>& toggleRects,
                     bool exitStatus, bool recordStatus, bool depthStatus, bool cloudStatus)
{
    toggle_states[0] = exitStatus;
    toggle_states[1] = recordStatus;
    toggle_states[2] = depthStatus;
    toggle_states[3] = cloudStatus;

    drawRoundedRect(img, Rect(x, y, width, height), PANEL_COLOR, 10, 1, true);
    drawShadowText(img, "Control Panel", Point(x + 15, y + 25),
                  FONT_HERSHEY_SIMPLEX, 0.7, TEXT_COLOR, 2);
    line(img, Point(x + 10, y + 35), Point(x + width - 10, y + 35),
         Scalar(100, 100, 100), 1);
    int btnX = x + 30;
    int btnY = y + 55;
    int btnWidth = 80;
    int btnHeight = 32;
    int btnSpacing = 210;

    toggleRects.clear();
    Rect exitRect(btnX, btnY, btnWidth, btnHeight);
    toggleRects.push_back(exitRect);
    drawToggleButton(img, exitRect, "ExitProgram", exitStatus, 0, toggle_clicked);

    Rect recordRect(btnX + btnSpacing, btnY, btnWidth, btnHeight);
    toggleRects.push_back(recordRect);
    drawToggleButton(img, recordRect, "RecordVideo", recordStatus, 1, toggle_clicked);

    Rect depthRect(btnX + 2*btnSpacing, btnY, btnWidth, btnHeight);
    toggleRects.push_back(depthRect);
    drawToggleButton(img, depthRect, "DepthCover", depthStatus, 2, toggle_clicked);

    Rect cloudRect(btnX + 3*btnSpacing, btnY, btnWidth, btnHeight);
    toggleRects.push_back(cloudRect);
    drawToggleButton(img, cloudRect, "PointCloud", cloudStatus, 3, toggle_clicked);

    Rect uartTestRect(btnX + 4*btnSpacing, btnY, btnWidth, btnHeight);
    toggleRects.push_back(uartTestRect);
    drawToggleButton(img, uartTestRect, "UARTTest", uart_test_enabled, 4, toggle_clicked);

    Rect byteTrackerRect(btnX + 5*btnSpacing, btnY, btnWidth, btnHeight);
    toggleRects.push_back(byteTrackerRect);
    drawToggleButton(img, byteTrackerRect, "ByteTracker", bytetracker_enabled, 5, toggle_clicked);
}

void drawStatusBar(Mat& img, int x, int y, int width, int height, const string& mapName, int enemyType)
{
    drawRoundedRect(img, Rect(x, y, width, height), THEME_COLOR, 10, 1, true);

    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::stringstream timeStr;
    timeStr << std::put_time(std::localtime(&now_time), "%H:%M:%S");
    drawShadowText(img, timeStr.str(), Point(x + width - 100, y + height/2 + 5),
                  FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);

    string teamText = enemyType == 1 ? "TEAM: RED" : "TEAM: BLUE";
    Scalar teamColor = enemyType == 1 ? TEAM_RED : TEAM_BLUE;
    int baseline;
    Size textSize = getTextSize(teamText, FONT_HERSHEY_SIMPLEX, 0.8, 2, &baseline);
    drawShadowText(img, teamText,
                  Point(x + width/2 - textSize.width/2, y + height/2 + 5),
                  FONT_HERSHEY_SIMPLEX, 0.8, teamColor, 2);

    drawShadowText(img, "MAP: " + mapName, Point(x + 300, y + height/2 + 5),
                  FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);

    drawStatusIndicator(img, Point(x + 20, y + height/2), true);
    drawShadowText(img, "Radar System", Point(x + 35, y + height/2 + 5),
                  FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);
}

void drawMapMarker(Mat& img, Point center, int id, bool isBlue, int size = 40)
{
    Scalar color = isBlue ? TEAM_BLUE : TEAM_RED;

    for (int i = 3; i >= 1; i--) {
        circle(img, center, size + i, color, 1);
    }
    circle(img, center, size, color, 2);

    string text = (isBlue ? "B" : "R") + to_string(id);
    int baseline;
    double fontScale = 2.0;
    Size text_size = getTextSize(text, FONT_HERSHEY_SIMPLEX, fontScale, 3, &baseline);
    Point textPos(center.x - text_size.width / 2, center.y + text_size.height / 2);

    Rect textRect(textPos.x - 10, textPos.y - text_size.height - 10,
                 text_size.width + 20, text_size.height + 20);
    Mat overlay;
    img.copyTo(overlay);
    rectangle(overlay, textRect, BG_COLOR, -1);
    addWeighted(overlay, 0.7, img, 0.3, 0, img);

    drawShadowText(img, text, textPos, FONT_HERSHEY_SIMPLEX, fontScale, color, 3);
}

void drawMapArea(Mat& img, const Mat& mapImage, Rect mapRect, const vector<radar2025::Location>& locations, int enemyType)
{
    drawRoundedRect(img, mapRect, PANEL_COLOR, 10, 1, true);
    drawShadowText(img, "Map Visualization", Point(mapRect.x + 15, mapRect.y + 25),
                  FONT_HERSHEY_SIMPLEX, 0.7, TEXT_COLOR, 2);

    if (!mapImage.empty()) {
        Rect innerMapRect(mapRect.x + 10, mapRect.y + 40,
                        mapRect.width - 20, mapRect.height - 80);

        Mat resizedMap;
        double mapAspectRatio = static_cast<double>(mapImage.cols) / mapImage.rows;
        double displayAspectRatio = static_cast<double>(innerMapRect.width) / innerMapRect.height;
        int finalWidth, finalHeight;
        int offsetX = 0, offsetY = 0;
        if (mapAspectRatio > displayAspectRatio) {
            finalWidth = innerMapRect.width;
            finalHeight = static_cast<int>(finalWidth / mapAspectRatio);
            offsetY = (innerMapRect.height - finalHeight) / 2;
        } else {
            finalHeight = innerMapRect.height;
            finalWidth = static_cast<int>(finalHeight * mapAspectRatio);
            offsetX = (innerMapRect.width - finalWidth) / 2;
        }

        Mat background = Mat::zeros(innerMapRect.height, innerMapRect.width, CV_8UC3);
        background.setTo(BG_COLOR);
        resize(mapImage, resizedMap, Size(finalWidth, finalHeight));

        rectangle(img, innerMapRect, Scalar(100, 100, 100), 1);
        background.copyTo(img(innerMapRect));
        Rect mapROI(innerMapRect.x + offsetX, innerMapRect.y + offsetY, finalWidth, finalHeight);
        resizedMap.copyTo(img(mapROI));
        if ((bytetracker_enabled || trajectory_fade_counter > 0) && !trajectories.empty()) {
            float alpha = bytetracker_enabled ? 1.0f : (float)trajectory_fade_counter / TRAJECTORY_FADE_FRAMES;
            for (const auto& traj_pair : trajectories) {
                const Trajectory& traj = traj_pair.second;
                if (traj.size() < 2) continue;
                Scalar traj_color;
                int id = traj_pair.first;
                if (id < 6) {
                    traj_color = TEAM_BLUE;
                } else {
                    traj_color = TEAM_RED;
                }
                for (size_t i = 1; i < traj.size(); i++) {
                    float segment_alpha = alpha * (0.3f + 0.7f * (float)i / traj.size());
                    Scalar color = traj_color * segment_alpha;
                    Point2f p1 = traj[i-1];
                    Point2f p2 = traj[i];
                    if (enemyType == 1) {
                        p1.x = 15.0f - p1.x;
                        p1.y = 28.0f - p1.y;
                        p2.x = 15.0f - p2.x;
                        p2.y = 28.0f - p2.y;
                    }
                    Point map_p1(
                        (p1.x / 15.0f) * finalWidth + innerMapRect.x + offsetX,
                        (p1.y / 28.0f) * finalHeight + innerMapRect.y + offsetY
                    );
                    Point map_p2(
                        (p2.x / 15.0f) * finalWidth + innerMapRect.x + offsetX,
                        (p2.y / 28.0f) * finalHeight + innerMapRect.y + offsetY
                    );
                    line(img, map_p1, map_p2, color, 2 + i/2, LINE_AA);
                }
            }
        }
        for (auto it : locations) {
            if (it.x == 0 && it.y == 0)
                continue;
            float x = it.x;
            float y = it.y;
            if (enemyType == 1) {
                x = 28.0 - x;
                y = 15.0 - y;
            }
            Point2f center = Point2f(
                (y / 15.0) * finalWidth + innerMapRect.x + offsetX,
                (x / 28.0) * finalHeight + innerMapRect.y + offsetY
            );
            Scalar color;
            string text;
            if (it.id < 6) {
                color = TEAM_BLUE; 
                text += "B";
            } else {
                color = TEAM_RED; 
                text += "R";
            }
            text += to_string(it.id);
            circle(img, center, 40.0, color, 3);
            int baseline;
            Size text_size = getTextSize(text, FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);
            center.x = center.x - text_size.width / 2;
            center.y = center.y + (text_size.height) / 2;
            putText(img, text, center, FONT_HERSHEY_SIMPLEX, 1, color, 2);
        }
    } else {
        drawShadowText(img, "Map not available", 
                      Point(mapRect.x + mapRect.width/2 - 80, mapRect.y + mapRect.height/2),
                      FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
    }
}
void drawUARTTestInfoPanel(Mat& img, int x, int y, int width, int height, vector<Rect>& toggleRects)
{
    if (!show_uart_test_info) return;
    drawRoundedRect(img, Rect(x, y, width, height), Scalar(40, 40, 60), 10, 1, true);
    drawShadowText(img, "UART Test Info", Point(x + 15, y + 25), 
                  FONT_HERSHEY_SIMPLEX, 0.7, ACCENT_COLOR, 2);
    line(img, Point(x + 10, y + 35), Point(x + width - 10, y + 35), 
         Scalar(100, 100, 100), 1);
    drawStatusIndicator(img, Point(x + 20, y + 55), uart_test_enabled);
    drawShadowText(img, "Status: " + string(uart_test_enabled ? "Active" : "Inactive"), 
                  Point(x + 40, y + 60), FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);
    drawShadowText(img, "Interval: " + to_string(uart_test_interval) + " ms (" + to_string(1000/uart_test_interval) + " Hz)", 
                  Point(x + 20, y + 90), FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);
    drawShadowText(img, "Protocol: 0x0305 (Map Robot Position)", 
                  Point(x + 20, y + 120), FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);
    drawShadowText(img, "Format: All robots in single packet", 
                  Point(x + 20, y + 150), FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);
    drawShadowText(img, "Coordinates: uint16 (centimeters)", 
                  Point(x + 20, y + 180), FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gui_node");
    ros::NodeHandle nh;
    g_nh = &nh; 
    std::string share_path = ros::package::getPath("radar2025");
    ros::Subscriber msg_sub = nh.subscribe("/radar2025/locations", 1, locations_msgCallback);
    image_transport::ImageTransport it_(nh);
    image_transport::Subscriber image_sub_;
    image_sub_ = it_.subscribe("/radar2025/result_view", 1, image_msgCallback);
    namedWindow("GUI", WINDOW_GUI_NORMAL);
    vector<Rect> toggleRects;
    setMouseCallback("GUI", mouseCallback, &toggleRects);
    bool if_exit_program = false;
    nh.setParam("/gui/CoverDepth", true);
    bool _if_record = true;
    bool _if_coverDepth = true;
    bool _if_showPointCloud = false;
    nh.setParam("/gui/ShowPointCloud", _if_showPointCloud);
    nh.setParam("/radar2025/UARTTestEnabled", uart_test_enabled);
    if (!nh.getParam("/radar2025/uart_test/interval", uart_test_interval)) {
        if (!nh.getParam("/radar2025/UARTTestInterval", uart_test_interval)) {
            uart_test_interval = 100; 
        }
    }
    uart_test_interval = std::min(std::max(uart_test_interval, 100), 1000); 
    if (!nh.getParam("/gui/UseByteTracker", bytetracker_enabled)) {
        if (!nh.getParam("/radar2025/bytetracker/enable", bytetracker_enabled)) {
            bytetracker_enabled = false; 
        }
        nh.setParam("/gui/UseByteTracker", bytetracker_enabled);
    }
    toggle_states[5] = bytetracker_enabled;
    string param_name, map_name;
    int ENEMY = 0;
    if (nh.searchParam("/gui/CoverDepth", param_name))
    {
        nh.getParam(param_name, _if_coverDepth);
        toggle_states[2] = _if_coverDepth;
    }
    else
    {
        ROS_WARN("Parameter CoverDepth not defined");
    }
    if (nh.searchParam("/gui/MapRMUC", param_name))
    {
        nh.getParam(param_name, map_name);
    }
    else
    {
        ROS_WARN("Parameter MapRMUC not defined");
        map_name = "Unknown";
    }
    if (nh.searchParam("/radar2025/EnemyType", param_name))
    {
        nh.getParam(param_name, ENEMY);
    }
    else
    {
        ROS_WARN("Parameter EnemyType not defined");
    }
    Mat map;
    if (!map_name.empty() && access((share_path + "/resources/" + map_name).c_str(), F_OK) == 0)
    {
        map = cv::imread(share_path + "/resources/" + map_name);
        if (ENEMY == 1)
            cv::flip(map, map, -1);
    }
    else
    {
        ROS_WARN("Map file not found!");
    }
    while (ros::ok() && !if_exit_program)
    {
        ros::spinOnce();
        if (!result_image || result_image->image.empty())
            continue;
        cv::Size originalSize = result_image->image.size();
        double scaleFactor = 1.0;
        if (originalSize.width > 1920) {
            scaleFactor = min(1920.0 / originalSize.width, 1080.0 / originalSize.height);
            scaleFactor = max(scaleFactor, 0.25);
        }
        scaleFactor = max(scaleFactor, 0.25);
        int adjustedWidth = static_cast<int>(originalSize.width * scaleFactor);
        int adjustedHeight = static_cast<int>(originalSize.height * scaleFactor);
        int mapWidth = min(700, static_cast<int>(adjustedHeight * 0.8)); 
        int canvasWidth = min(1920, adjustedWidth + mapWidth);
        int canvasHeight = min(1080, adjustedHeight);
        Mat display = Mat::zeros(Size(canvasWidth, canvasHeight), CV_8UC3);
        static cv::Size lastSize(0, 0);
        if (lastSize != originalSize) {
            lastSize = originalSize;
            ROS_INFO("Camera resolution: %dx%d, Scaling to: %dx%d, Canvas: %dx%d", 
                     originalSize.width, originalSize.height, 
                     adjustedWidth, adjustedHeight,
                     canvasWidth, canvasHeight);
        }
        display.setTo(BG_COLOR);
        int margin = 10;
        int statusBarHeight = 40;
        int controlPanelHeight = 100;
        Rect statusBarRect(margin, margin, 
                          canvasWidth - 2*margin, 
                          statusBarHeight);
        Rect cameraRect(margin, 
                       statusBarRect.y + statusBarRect.height + margin,
                       adjustedWidth - 2*margin,
                       canvasHeight - statusBarHeight - controlPanelHeight - 3*margin);
        Rect mapRect(cameraRect.x + cameraRect.width + margin,
                    statusBarRect.y + statusBarRect.height + margin,
                    canvasWidth - cameraRect.width - 3*margin,
                    canvasHeight - statusBarHeight - controlPanelHeight - 3*margin);
        Rect controlPanelRect(margin,
                             canvasHeight - controlPanelHeight - margin,
                             canvasWidth - 2*margin,
                             controlPanelHeight);
        drawStatusBar(display, statusBarRect.x, statusBarRect.y, 
                     statusBarRect.width, statusBarRect.height, 
                     map_name, ENEMY);
        drawRoundedRect(display, cameraRect, PANEL_COLOR, 10, 1, true);
        drawShadowText(display, "Camera View", Point(cameraRect.x + 15, cameraRect.y + 25),
                      FONT_HERSHEY_SIMPLEX, 0.7, TEXT_COLOR, 2);
        Rect innerCameraRect(cameraRect.x + 10, cameraRect.y + 40, 
                           cameraRect.width - 20, cameraRect.height - 50);
        Mat resizedImage;
        if (result_image->image.cols > 3000 || result_image->image.rows > 2000) {
            double targetWidth = innerCameraRect.width;
            double targetHeight = innerCameraRect.height;
            double scaleX = targetWidth / result_image->image.cols;
            double scaleY = targetHeight / result_image->image.rows;
            double scale = std::min(scaleX, scaleY); 
            Mat tempImage;
            int intermediateWidth = std::max(800, int(result_image->image.cols * scale * 1.5));
            int intermediateHeight = intermediateWidth * result_image->image.rows / result_image->image.cols;
            cv::resize(result_image->image, tempImage, 
                      Size(intermediateWidth, intermediateHeight), 
                      0, 0, INTER_AREA);
            cv::resize(tempImage, resizedImage, 
                      Size(innerCameraRect.width, innerCameraRect.height), 
                      0, 0, INTER_LINEAR);
            tempImage.release();
            static cv::Size lastProcessedSize;
            if (lastProcessedSize != result_image->image.size()) {
                ROS_INFO("High resolution image detected (%dx%d), using optimized 2-step scaling", 
                         result_image->image.cols, result_image->image.rows);
                lastProcessedSize = result_image->image.size();
            }
        } else {
            cv::resize(result_image->image, resizedImage, 
                      Size(innerCameraRect.width, innerCameraRect.height), 
                      0, 0, INTER_AREA);
        }
        resizedImage.copyTo(display(innerCameraRect));
        if (toggle_states[3]) { 
            int infoX = cameraRect.x + cameraRect.width - 220; 
            int infoY = cameraRect.y + 70; 
            int infoWidth = 200; 
            int infoHeight = 100;
            Mat overlay;
            display.copyTo(overlay);
            Rect infoRect(infoX, infoY, infoWidth, infoHeight);
            rectangle(overlay, infoRect, Scalar(20, 20, 20), -1);
            addWeighted(overlay, 0.7, display, 0.3, 0, display);
            drawShadowText(display, "Point Cloud Info", 
                          Point(infoX + 10, infoY + 20), 
                          FONT_HERSHEY_SIMPLEX, 0.5, ACCENT_COLOR, 1);
            drawShadowText(display, "Status: Active", 
                          Point(infoX + 10, infoY + 45), 
                          FONT_HERSHEY_SIMPLEX, 0.45, STATUS_GREEN, 1);
            drawShadowText(display, "Point Size: 2 px", 
                          Point(infoX + 10, infoY + 70), 
                          FONT_HERSHEY_SIMPLEX, 0.45, TEXT_COLOR, 1);
            drawShadowText(display, "Auto Range: ON", 
                          Point(infoX + 10, infoY + 95), 
                          FONT_HERSHEY_SIMPLEX, 0.45, TEXT_COLOR, 1);
        }

        drawMapArea(display, map, mapRect, locs, ENEMY);
        drawControlPanel(display, controlPanelRect.x, controlPanelRect.y,
                        controlPanelRect.width, controlPanelRect.height,
                        toggleRects,
                        toggle_states[0], toggle_states[1], toggle_states[2], toggle_states[3]);

        if (show_uart_test_info) {
            int uartInfoX = mapRect.x + 10;
            int uartInfoY = mapRect.y + 40;
            int uartInfoWidth = mapRect.width - 20;
            int uartInfoHeight = 210;
            drawUARTTestInfoPanel(display, uartInfoX, uartInfoY, uartInfoWidth, uartInfoHeight, toggleRects);
        }

        cv::imshow("GUI", display);

        int maxDisplayWidth = std::min(1920, canvasWidth);
        int maxDisplayHeight = std::min(1080, canvasHeight);
        if (maxDisplayWidth > 1536) {
            double ratio = 1536.0 / maxDisplayWidth;
            maxDisplayWidth = 1536;
            maxDisplayHeight = static_cast<int>(maxDisplayHeight * ratio);
        }
        cv::resizeWindow("GUI", Size(maxDisplayWidth, maxDisplayHeight));

        static bool firstDisplay = true;
        if (firstDisplay) {
            cv::moveWindow("GUI", 100, 50);
            firstDisplay = false;
        }

        if (toggle_states[0] != if_exit_program) {
            if_exit_program = toggle_states[0];
            nh.setParam("/radar2025/ExitProgram", if_exit_program);
        }
        if (toggle_states[1] != _if_record) {
            _if_record = toggle_states[1];
            nh.setParam("/radar2025/Recorder", _if_record);
        }
        if (toggle_states[2] != _if_coverDepth) {
            _if_coverDepth = toggle_states[2];
            nh.setParam("/gui/CoverDepth", _if_coverDepth);
        }
        if (toggle_states[3] != _if_showPointCloud) {
            _if_showPointCloud = toggle_states[3];
            nh.setParam("/gui/ShowPointCloud", _if_showPointCloud);
        }

        static bool previous_test_state = false;
        if (uart_test_enabled != previous_test_state) {
            previous_test_state = uart_test_enabled;
            nh.setParam("/radar2025/UARTTestEnabled", uart_test_enabled);
            if (uart_test_enabled) {
                nh.setParam("/radar2025/UARTTestInterval", uart_test_interval);
            }
        }

        static bool previous_bytetracker_state = bytetracker_enabled;
        if (bytetracker_enabled != previous_bytetracker_state) {
            previous_bytetracker_state = bytetracker_enabled;
            nh.setParam("/gui/UseByteTracker", bytetracker_enabled);
        }

        cv::waitKey(1);
    }

    cv::destroyAllWindows();
    ros::shutdown();
    return 0;
}
