/*
For class E502 Introduction to Cyber Physical Systems
ISE, Indiana University
Lantao Liu
1/14/2019
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_demo");

  //create a ros handle 
  ros::NodeHandle n;

  // <>: it specified the type of the message to be published
  // (): first param: topic name; second param: size of queued messages, at least 1
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("some_chatter", 10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  //each second, ros "spins" and draws 20 frames
  ros::Rate loop_rate(20);

  int frame_count = 0;
  float f = 0.0;
  int width = 10;

  while (ros::ok())
  {
    //first create a string typed (std_msgs) message
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Frame index: " << frame_count; // "Frame index: number_of_frame_count"
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str()); //printing on screen

    //publisher publishes messages, the msg type must be consistent with definition advertise<>();
    chatter_pub.publish(msg);



  /******************** From here, we are defining and drawing Boundary and obstacles in the workspace **************************/

    visualization_msgs::Marker obstBoundary;
    obstBoundary.type = visualization_msgs::Marker::LINE_STRIP; //Set obstBoundary as a line strip
    // Set the frame id and timestamp
    obstBoundary.header.frame_id = "map"; //NOTE: this should be "paired" to the frame_id entry in Rviz
    obstBoundary.header.stamp = ros::Time::now();

    // Set the namespace and id
    obstBoundary.ns = "Boundary";
    obstBoundary.id = 0;

    // Set the marker action
    obstBoundary.action = visualization_msgs::Marker::ADD;

    //Set the scale of the marker. For line strip there is only x width available
    obstBoundary.scale.x = 0.2;

    // Set the color and transparency for the marker. If not set default values are 0 for all the fields.
    obstBoundary.color.r = 1.0f;
    obstBoundary.color.g = 1.0f;
    obstBoundary.color.b = 0.0f;
    obstBoundary.color.a = 1.0; //be sure to set alpha to something non-zero, otherwise it is transparent

    obstBoundary.lifetime = ros::Duration();


    geometry_msgs::Point p;

    p.x = -width/2;
    p.y = -width/2;
    p.z = 0;

    obstBoundary.points.push_back(p);
    p.x = width/2;
    obstBoundary.points.push_back(p);
    p.y = width/2;
    obstBoundary.points.push_back(p);
    p.x = -width/2;
    obstBoundary.points.push_back(p);
    p.y = -width/2;
    obstBoundary.points.push_back(p);

    // Publish the arena boundry (represented by LINE_STRIP marker) to ROS system
    marker_pub.publish(obstBoundary);

    //Create array of Visualization markers
    visualization_msgs::Marker obst [7];

    // 4 obstacles shaped as CUBE (ID: 0,1,2,3)
    for (int i = 0; i<4; i++) {
      obst[i].type = visualization_msgs::Marker::CUBE;
      obst[i].header.frame_id = "map";
      obst[i].header.stamp = ros::Time::now();
      obst[i].ns = "obstacles";
      obst[i].id = i;
      obst[i].action = visualization_msgs::Marker::ADD;
      obst[i].color = obstBoundary.color;

      //Set obstacle scale
      obst[i].scale.x = obst[i].scale.z = 0.5;
      obst[i].scale.y = 8;
      obst[i].pose.position.z = 0.25;
      obst[i].pose.orientation.x = obst[i].pose.orientation.y = obst[i].pose.orientation.z = 0;
      obst[i].pose.orientation.w = 1;
      switch (i) {
        case 0 : obst[i].pose.position.x = -3;
                 obst[i].pose.position.y = -1;
                 break;
        case 1 : obst[i].pose.position.x = -1.5;
                 obst[i].pose.position.y = 2;
                 obst[i].scale.y = 5;
                 obst[i].pose.orientation.w = obst[i].pose.orientation.z = 0.7071;
                 break;
        case 2 : obst[i].pose.position.x = 3;
                 obst[i].pose.position.y = 1;
                 break;
        case 3 : obst[i].pose.position.x = 1.5;
                 obst[i].pose.position.y = -2;
                 obst[i].scale.y = 5;
                 obst[i].pose.orientation.w = obst[i].pose.orientation.z = 0.7071;
                 break;
      }

      obst[i].lifetime = ros::Duration();
      marker_pub.publish(obst[i]);
    }


    // 3 obstacles shaped as CYLINDER (ID: 4,5,6)
    for (int i = 4; i<7; i++) {
      obst[i].type = visualization_msgs::Marker::CYLINDER;
      obst[i].header.frame_id = "map";
      obst[i].header.stamp = ros::Time::now();
      obst[i].ns = "obstacles";
      obst[i].id = i;
      obst[i].action = visualization_msgs::Marker::ADD;
      obst[i].color = obstBoundary.color;
      obst[i].scale.x = obst[i].scale.y = obst[i].scale.z = 2;
      obst[i].pose.position.x = 0;
      obst[i].pose.position.z = 1;

      switch (i) {
        case 4 : obst[i].pose.position.y = 4;
                 break;
        case 5 : obst[i].pose.position.y = 0;
                 break;
        case 6 : obst[i].pose.position.y = -4;
                 break;
      }
      obst[i].lifetime = ros::Duration();
      marker_pub.publish(obst[i]);
    }


    /******************** Defining start point and goal points *******************/

    visualization_msgs::Marker GoalPoint;
    GoalPoint.type = visualization_msgs::Marker::POINTS;

    GoalPoint.header.frame_id = "map";
    GoalPoint.header.stamp =  ros::Time::now();
    GoalPoint.ns =  "Goal Points";
    GoalPoint.action =  visualization_msgs::Marker::ADD;
    GoalPoint.pose.orientation.w = 1.0;

    GoalPoint.id = 0;
    GoalPoint.scale.x = 0.25;
    GoalPoint.scale.y = 0.25;
    GoalPoint.color.g = 1.0f;
    GoalPoint.color.a = 1.0;

    geometry_msgs::Point point;	// root vertex
    point.z = 0;
    point.x = -4;
    point.y = -4;
    GoalPoint.points.push_back(point);

    point.x = 4;
    point.y = 4;
    GoalPoint.points.push_back(point);
    GoalPoint.lifetime = ros::Duration();
    marker_pub.publish(GoalPoint);


  /************************* From here, we are using points, lines, to draw dynamically *** ******************/

    //we use static here since we want to incrementally add contents in these mesgs, otherwise contents in these msgs will be cleaned in every ros spin.
    static visualization_msgs::Marker vertices, edges;

    vertices.type = visualization_msgs::Marker::POINTS;
    edges.type = visualization_msgs::Marker::LINE_LIST;

    vertices.header.frame_id = edges.header.frame_id = "map";
    vertices.header.stamp = edges.header.stamp = ros::Time::now();
    vertices.ns = edges.ns = "vertices_and_lines";
    vertices.action = edges.action = visualization_msgs::Marker::ADD;
    vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;

    vertices.id = 0;
    edges.id = 1;

    // POINTS markers use x and y scale for width/height respectively
    vertices.scale.x = 0.05;
    vertices.scale.y = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    edges.scale.x = 0.02; //tune it yourself

    // Points are green
    vertices.color.g = 1.0f;
    vertices.color.a = 1.0;

    // Line list is red
    edges.color.r = 1.0;
    edges.color.a = 1.0;

    geometry_msgs::Point p0;	// root vertex
    p0.x = p0.y = 0;
    p0.z = 2;
    int num_slice = 20;		// e.g., to create a point for each 20 edges 
    float length = 1;		//length of each edge

    static int slice_index = 0;
    int herz = 10;		//every 10 ROS frames we draw an edge
    if(frame_count % herz == 0 && edges.points.size()<= 2*num_slice)
    {
      geometry_msgs::Point p;

      float angle = slice_index*2*M_PI/num_slice;
      slice_index ++ ;
      p.x = length * cos(angle);
      p.y = length * sin(angle);
      p.z = 2;

      vertices.points.push_back(p);	//for drawing vertices
      edges.points.push_back(p0);	//for drawing edges. The line list needs two points for each line
      edges.points.push_back(p);
    }

    //publish msgs
    marker_pub.publish(vertices);
    marker_pub.publish(edges);


  /******************** From here, we are defining and drawing a simple robot **************************/

    // a simple sphere represents a robot
    static visualization_msgs::Marker rob;
    static visualization_msgs::Marker path;
    rob.type = visualization_msgs::Marker::SPHERE;
    path.type = visualization_msgs::Marker::LINE_STRIP;

    rob.header.frame_id = path.header.frame_id = "map";  //NOTE: this should be "paired" to the frame_id entry in Rviz, the default setting in Rviz is "map"
    rob.header.stamp = path.header.stamp = ros::Time::now();
    rob.ns = path.ns = "rob";
    rob.id = 0;
    path.id = 1;
    rob.action = path.action = visualization_msgs::Marker::ADD;
    rob.lifetime = path.lifetime = ros::Duration();

    rob.scale.x = rob.scale.y = rob.scale.z = 0.3;

    rob.color.r = 1.0f;
    rob.color.g = 0.5f;
    rob.color.b = 0.5f;
    rob.color.a = 1.0;

    // path line strip is blue
    path.color.b = 1.0;
    path.color.a = 1.0;

    path.scale.x = 0.02;
    path.pose.orientation.w = 1.0;

    int num_slice2 = 200;		// divide a circle into segments
    static int slice_index2 = 0;
    if(frame_count % 2 == 0 && path.points.size() <= num_slice2)  //update every 2 ROS frames
    {
      geometry_msgs::Point p;

      float angle = slice_index2*2*M_PI/num_slice2;
      slice_index2 ++ ;
      p.x = 4 * cos(angle) - 0.5;  	//some random circular trajectory, with radius 4, and offset (-0.5, 1, .05)
      p.y = 4 * sin(angle) + 1.0;
      p.z = 0.05;

      rob.pose.position = p;
      path.points.push_back(p);		//for drawing path, which is line strip type
    }

    marker_pub.publish(rob);
    marker_pub.publish(path);


  /******************** To here, we finished displaying our components **************************/

    // check if there is a subscriber. Here our subscriber will be Rviz
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please run Rviz in another terminal.");
      sleep(1);
    }


    //ros spins, force ROS frame to refresh/update once
    ros::spinOnce();

    loop_rate.sleep();
    ++frame_count;
  }

  return 0;
}
