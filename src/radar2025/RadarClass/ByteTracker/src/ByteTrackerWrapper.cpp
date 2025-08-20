#include "../include/ByteTrackerWrapper.h"

ByteTrackerWrapper::ByteTrackerWrapper() 
    : is_enabled(false), 
      frame_rate(30),
      track_buffer(30),
      track_thresh(0.5),
      high_thresh(0.6),
      match_thresh(0.8),
      track_counter(0),
      track_success_rate(0.0f),
      detection_processed(0),
      tracks_generated(0)
{
    logger->info("ByteTrackerWrapper created");
}

ByteTrackerWrapper::~ByteTrackerWrapper()
{
    if (detection_processed > 0) {
        logger->info("ByteTracker statistics: Processed {} detections, generated {} tracks, success rate: {:.2f}%", 
                     detection_processed, tracks_generated, 
                     (detection_processed > 0 ? track_success_rate * 100.0f / track_counter : 0.0f));
    }
    logger->info("ByteTrackerWrapper destroyed");
}

bool ByteTrackerWrapper::init(int frame_rate, int track_buffer, float track_thresh, float high_thresh, float match_thresh)
{
    try {
        this->frame_rate = frame_rate;
        this->track_buffer = track_buffer;
        this->track_thresh = track_thresh;
        this->high_thresh = high_thresh;
        this->match_thresh = match_thresh;
        
        tracker = std::make_unique<byte_track::BYTETracker>(
            frame_rate, track_buffer, track_thresh, high_thresh, match_thresh);
        
        logger->info("ByteTracker initialized with parameters: frame_rate={}, track_buffer={}, track_thresh={:.2f}, high_thresh={:.2f}, match_thresh={:.2f}",
                    frame_rate, track_buffer, track_thresh, high_thresh, match_thresh);
        
        track_counter = 0;
        track_success_rate = 0.0f;
        detection_processed = 0;
        tracks_generated = 0;
        
        track_history.clear();
        
        is_enabled = true;
        return true;
    } catch (const std::exception& e) {
        logger->error("Failed to initialize ByteTracker: {}", e.what());
        is_enabled = false;
        return false;
    }
}

void ByteTrackerWrapper::setEnabled(bool enabled)
{
    if (is_enabled != enabled) {
        is_enabled = enabled;
        logger->info("ByteTracker {} enabled", enabled ? "is" : "is not");
        
        if (enabled && tracker) {
            tracker = std::make_unique<byte_track::BYTETracker>(
                frame_rate, track_buffer, track_thresh, high_thresh, match_thresh);
            
            track_counter = 0;
            track_success_rate = 0.0f;
            detection_processed = 0;
            tracks_generated = 0;
            
            track_history.clear();
            
            logger->info("ByteTracker reinitialized");
        }
    }
}

bool ByteTrackerWrapper::isEnabled() const
{
    return is_enabled;
}

vector<bboxAndRect> ByteTrackerWrapper::track(const vector<bboxAndRect>& detections)
{
    if (!is_enabled || !tracker) {
        return detections;
    }
    
    try {
        detection_processed += detections.size();
        
        std::vector<byte_track::Object> objects;
        std::map<size_t, bboxAndRect> index_to_detection;
        
        for (size_t i = 0; i < detections.size(); ++i) {
            objects.push_back(convertToObject(detections[i]));
            index_to_detection[i] = detections[i];
        }
        
        auto tracks = tracker->update(objects);
        tracks_generated += tracks.size();
        track_counter++;
        
        float success_rate = detections.size() > 0 ? static_cast<float>(tracks.size()) / detections.size() : 0.0f;
        track_success_rate += success_rate;
        
        vector<bboxAndRect> tracked_detections;
        std::map<int, bboxAndRect> current_tracks;
        
        for (size_t i = 0; i < tracks.size(); ++i) {
            const auto& track = tracks[i];
            int track_id = static_cast<int>(track->getTrackId());
            
            float best_iou = 0.0f;
            bboxAndRect best_match;
            bool match_found = false;
            
            for (const auto& [idx, detection] : index_to_detection) {
                byte_track::Rect<float> track_rect = track->getRect();
                byte_track::Rect<float> det_rect(
                    detection.armor.x0, 
                    detection.armor.y0, 
                    detection.armor.w, 
                    detection.armor.h
                );
                
                float iou = track_rect.calcIoU(det_rect);
                if (iou > best_iou) {
                    best_iou = iou;
                    best_match = detection;
                    match_found = true;
                }
            }
            
            if (match_found && best_iou > 0.1f) {
                bboxAndRect tracked_det = best_match;
                tracked_det.rect.trackID = track_id;
                const auto& rect = track->getRect();
                const float alpha = 0.7f;
                tracked_det.armor.x0 = alpha * best_match.armor.x0 + (1-alpha) * rect.x();
                tracked_det.armor.y0 = alpha * best_match.armor.y0 + (1-alpha) * rect.y();
                tracked_det.armor.w = alpha * best_match.armor.w + (1-alpha) * rect.width();
                tracked_det.armor.h = alpha * best_match.armor.h + (1-alpha) * rect.height();
                current_tracks[track_id] = tracked_det;
                tracked_detections.push_back(tracked_det);
            }
        }
        
        track_history = current_tracks;
        
        if (track_counter % 100 == 0) {
            logger->info("ByteTracker: Processed {} frames, current success rate: {:.2f}%", 
                         track_counter, 
                         (track_counter > 0 ? track_success_rate * 100.0f / track_counter : 0.0f));
        }
        
        logger->debug("ByteTracker: Tracking {} detections resulted in {} tracked objects", 
                    detections.size(), tracked_detections.size());
        
        return tracked_detections;
    } catch (const std::exception& e) {
        logger->error("Exception in ByteTracker::track: {}", e.what());
        return detections;
    }
}

byte_track::Object ByteTrackerWrapper::convertToObject(const bboxAndRect& detection)
{
    float x = std::max(0.0f, detection.armor.x0);
    float y = std::max(0.0f, detection.armor.y0);
    float w = std::max(1.0f, detection.armor.w);
    float h = std::max(1.0f, detection.armor.h);
    
    byte_track::Rect<float> rect(x, y, w, h);
    
    return byte_track::Object(
        rect, 
        static_cast<int>(detection.armor.cls),
        detection.armor.conf
    );
}

bboxAndRect ByteTrackerWrapper::convertToDetection(
    const std::shared_ptr<byte_track::STrack>& track, 
    const bboxAndRect& original_detection)
{
    bboxAndRect result = original_detection;
    result.rect.trackID = static_cast<int>(track->getTrackId());
    const auto& rect = track->getRect();
    const float alpha = 0.7f;
    result.armor.x0 = alpha * original_detection.armor.x0 + (1-alpha) * rect.x();
    result.armor.y0 = alpha * original_detection.armor.y0 + (1-alpha) * rect.y();
    result.armor.w = alpha * original_detection.armor.w + (1-alpha) * rect.width();
    result.armor.h = alpha * original_detection.armor.h + (1-alpha) * rect.height();
    return result;
}

ByteTrackerStats ByteTrackerWrapper::getStats() const
{
    ByteTrackerStats stats;
    stats.tracks_processed = track_counter;
    stats.detections_processed = detection_processed;
    stats.tracks_generated = tracks_generated;
    stats.success_rate = track_counter > 0 ? track_success_rate / track_counter : 0.0f;
    return stats;
}
