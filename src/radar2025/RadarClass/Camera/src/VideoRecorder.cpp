#include "../include/VideoRecorder.h"
#include <sys/stat.h>
#include <sys/types.h>

#ifdef _WIN32
#include <direct.h>
#define mkdir(path, mode) _mkdir(path)
#endif

VideoRecorder::VideoRecorder()
{
}

VideoRecorder::~VideoRecorder()
{
    this->close();
}

bool VideoRecorder::init(const char *videoPath, int coder, Size size)
{
    if (access(videoPath, F_OK) == -1)
    {
        this->logger->error("[ERR] VideoPath, non-existent: {}", videoPath);
        this->logger->flush();
        if (mkdir(videoPath, 0755) == -1) {
            this->logger->error("Failed to create video directory");
            return false;
        } else {
            this->logger->info("Created video directory: {}", videoPath);
        }
    }
    
    if (!this->vw.isOpened())
    {
        char filename[1024];
        time_t currentTime = time(NULL);
        char chCurrentTime[256];
        strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d_%H%M%S", localtime(&currentTime));
        strcat(chCurrentTime, ".mp4");
        strcpy(filename, videoPath);
        int length = strlen(videoPath);
        if (videoPath[length - 1] != '/')
        {
            strcat(filename, "/");
        }
        strcat(filename, chCurrentTime);
        this->vw = VideoWriter();
        this->logger->info("Starting video recording to: {}", filename);
        if (!this->vw.open(filename, coder, 60.0, size, true))
        {
            this->logger->warn("Block Video Recorder - Failed to open video file");
            return false;
        }
        this->logger->info("Video recorder initialized with fixed FPS: 60.0");
    }
    return true;
}

void VideoRecorder::write(Mat src)
{
    if (!this->vw.isOpened())
        return;
    this->vw.write(src);
}

void VideoRecorder::close()
{
    if (this->vw.isOpened()) {
        this->logger->info("Closing video recording");
        this->vw.release();
    }
}