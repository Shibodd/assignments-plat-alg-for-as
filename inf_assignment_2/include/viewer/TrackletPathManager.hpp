#ifndef TRACKLET_PATH_MANAGER_HPP
#define TRACKLET_PATH_MANAGER_HPP

#include "tracker/Tracker.h"
#include "tracker/Tracklet.h"
#include "viewer/Path.hpp"
#include <map>

namespace viewer {

class TrackletPathManager {
  std::map<int, std::unique_ptr<Path>> paths_;

public:
  TrackletPathManager(Tracker& tracker) {
    register_to_tracker(tracker);
  }

  void onTracksUpdated(Tracklet& tracklet, bool added);
  void onHistoryUpdated(Tracklet& tracklet);

  std::vector<const Path*> getPaths();

  inline void register_to_tracker(Tracker& tracker) {
    tracker.setTracksUpdateListener([this](Tracklet& tracklet, bool added) {
      onTracksUpdated(tracklet, added);
    });
  }
};

}

#endif