#ifndef __BYTETRACKERWRAPPER_H
#define __BYTETRACKERWRAPPER_H

#include "../../Common/include/public.h"
#include <Eigen/Dense>
#include "ByteTrack/BYTETracker.h"
#include "ByteTrack/Object.h"
#include "ByteTrack/STrack.h"

struct ByteTrackerStats {
    int tracks_processed;
    int detections_processed;
    int tracks_generated;
    float success_rate;
};

class ByteTrackerWrapper
{
public:
    typedef std::shared_ptr<ByteTrackerWrapper> Ptr;

private:
    std::unique_ptr<byte_track::BYTETracker> tracker;
    bool is_enabled;
    int frame_rate;
    int track_buffer;
    float track_thresh;
    float high_thresh;
    float match_thresh;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    
    int track_counter;
    float track_success_rate;
    int detection_processed;
    int tracks_generated;
    
    std::map<int, bboxAndRect> track_history;

public:
    ByteTrackerWrapper();
    ~ByteTrackerWrapper();

    bool init(int frame_rate = 30, 
              int track_buffer = 30, 
              float track_thresh = 0.5, 
              float high_thresh = 0.6, 
              float match_thresh = 0.8);

    void setEnabled(bool enabled);
    bool isEnabled() const;

    vector<bboxAndRect> track(const vector<bboxAndRect>& detections);
    ByteTrackerStats getStats() const;

private:
    byte_track::Object convertToObject(const bboxAndRect& detection);
    bboxAndRect convertToDetection(const std::shared_ptr<byte_track::STrack>& track, 
                                  const bboxAndRect& original_detection);
};

#endif
