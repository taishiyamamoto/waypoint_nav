#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_srvs/Trigger.h>

#include "create_trigger_panel.h"

namespace waypoint_nav
{

CreateTriggerPanel::CreateTriggerPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  create_client_ = nh_.serviceClient<std_srvs::Trigger>("create_waypoints", false);

  create_button_ = new QPushButton("createWaypoints");

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget(create_button_);
  setLayout( layout );
  
  connect(create_button_, SIGNAL(clicked()), this, SLOT(pushCreateWaypoints()));
}

void CreateTriggerPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void CreateTriggerPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

void CreateTriggerPanel::pushCreateWaypoints() {
    ROS_INFO("Service call: create waypoints");
    
    std_srvs::Trigger trigger;
    if (create_client_.call(trigger)){
        ROS_INFO_STREAM("create success");
    } else {
        // ROS_WARN_STREAM(trigger.message);
    }
}

} // end namespace waypoint_nav

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(waypoint_nav::CreateTriggerPanel,rviz::Panel )