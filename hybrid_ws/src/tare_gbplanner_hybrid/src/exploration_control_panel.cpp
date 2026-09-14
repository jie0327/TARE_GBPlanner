#include "tare_gbplanner_hybrid/exploration_control_panel.h"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace tare_gbplanner_hybrid {

ExplorationControlPanel::ExplorationControlPanel(QWidget* parent)
    : rviz::Panel(parent),
      start_button_(new QPushButton("Start Navigation", this)),
      stop_button_(new QPushButton("Stop", this)),
      continue_button_(new QPushButton("Continue", this)),
      status_label_(new QLabel("Status: ready", this)) {
  start_publisher_ =
      node_handle_.advertise<std_msgs::Bool>("/start_exploration", 1, true);
  stop_publisher_ = node_handle_.advertise<std_msgs::Int8>("/stop", 1, true);

  stop_button_->setEnabled(false);
  continue_button_->setEnabled(false);

  auto* layout = new QVBoxLayout;
  layout->addWidget(new QLabel("Exploration control", this));
  layout->addWidget(start_button_);
  layout->addWidget(stop_button_);
  layout->addWidget(continue_button_);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  connect(start_button_, &QPushButton::clicked, this,
          &ExplorationControlPanel::startNavigation);
  connect(stop_button_, &QPushButton::clicked, this,
          &ExplorationControlPanel::stopNavigation);
  connect(continue_button_, &QPushButton::clicked, this,
          &ExplorationControlPanel::continueNavigation);
}

void ExplorationControlPanel::publishStopCommand(int value) {
  std_msgs::Int8 stop_message;
  stop_message.data = value;
  stop_publisher_.publish(stop_message);
}

void ExplorationControlPanel::setRunningState() {
  stop_button_->setEnabled(true);
  continue_button_->setEnabled(false);
  status_label_->setText("Status: exploring");
}

void ExplorationControlPanel::startNavigation() {
  publishStopCommand(0);
  std_msgs::Bool start_message;
  start_message.data = true;
  start_publisher_.publish(start_message);
  start_button_->setEnabled(false);
  setRunningState();
  ROS_INFO("RViz exploration control: start navigation");
}

void ExplorationControlPanel::stopNavigation() {
  std_msgs::Int8 stop_message;
  stop_message.data = 2;
  stop_publisher_.publish(stop_message);
  stop_button_->setEnabled(false);
  continue_button_->setEnabled(true);
  status_label_->setText("Status: stopped");
  ROS_INFO("RViz exploration control: full stop");
}

void ExplorationControlPanel::continueNavigation() {
  std_msgs::Int8 stop_message;
  stop_message.data = 0;
  stop_publisher_.publish(stop_message);
  setRunningState();
  ROS_INFO("RViz exploration control: continue navigation");
}

}  // namespace tare_gbplanner_hybrid

PLUGINLIB_EXPORT_CLASS(tare_gbplanner_hybrid::ExplorationControlPanel,
                       rviz::Panel)
