#ifndef CREATE_TRIGGER_PANEL_H
#define CREATE_TRIGGER_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

class QPushButton;

namespace waypoint_nav
{

class CreateTriggerPanel: public rviz::Panel
{
Q_OBJECT
public:
  CreateTriggerPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:
  void pushCreateWaypoints();
    
protected:
  ros::NodeHandle nh_;
  ros::ServiceClient create_client_;
  QPushButton *create_button_;
};

} // end namespace waypoint_nav

#endif // CREATE_TRIGGER_PANEL_H