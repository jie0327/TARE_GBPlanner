#ifndef TARE_GBPLANNER_HYBRID_EXPLORATION_CONTROL_PANEL_H
#define TARE_GBPLANNER_HYBRID_EXPLORATION_CONTROL_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>

class QLabel;
class QPushButton;

namespace tare_gbplanner_hybrid {

class ExplorationControlPanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit ExplorationControlPanel(QWidget* parent = nullptr);

 private Q_SLOTS:
  void startNavigation();
  void stopNavigation();
  void continueNavigation();

 private:
  void publishStopCommand(int value);
  void setRunningState();

  ros::NodeHandle node_handle_;
  ros::Publisher start_publisher_;
  ros::Publisher stop_publisher_;
  QPushButton* start_button_;
  QPushButton* stop_button_;
  QPushButton* continue_button_;
  QLabel* status_label_;
};

}  // namespace tare_gbplanner_hybrid

#endif  // TARE_GBPLANNER_HYBRID_EXPLORATION_CONTROL_PANEL_H
