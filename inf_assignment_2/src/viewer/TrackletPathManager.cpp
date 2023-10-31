#include "viewer/TrackletPathManager.hpp"
#include "logging.hpp"

static const logging::Logger logger("TrackletPathManager");

namespace viewer {

std::vector<const Path*> TrackletPathManager::getPaths() {
  std::vector<const Path*> v;
  v.reserve(paths_.size());
  for (const auto& path : paths_) {
    auto ptr = path.second.get();
    v.push_back(ptr);
  }
  return v;
}

void TrackletPathManager::onTracksUpdated(Tracklet& tracklet, bool added) {
  int id = tracklet.getId();

  if (added) {
    logger.debug("Registering tracklet %d.", id);
    
    auto res = paths_.emplace(id, std::make_unique<Path>(1.0, 0, 0, 1.0));

    if (not res.second)
      throw "The tracklet was already being managed by TrackletPathManager.";
    
    tracklet.setHistoryListener([this](Tracklet& tracklet) {
      onHistoryUpdated(tracklet);
    });
  } else {
    logger.debug("Deregistering tracklet %d.", id);

    int res = paths_.erase(id);
    if (res == 0)
      throw "The tracklet was not being managed by TrackletPathManager.";

    tracklet.setHistoryListener(nullptr);
  }
}

void TrackletPathManager::onHistoryUpdated(Tracklet& tracklet) {
  int id = tracklet.getId();

  logger.debug("Received history update from tracklet %d.", id);

  Eigen::Vector2d pos = tracklet.getLastPosition();

  paths_[id]->addPoint(Eigen::Vector3d(pos(0), pos(1), 0));
}
  
}