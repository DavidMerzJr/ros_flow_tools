#include <ros/ros.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <visualization_msgs/Marker.h>

// TODO: Implement class instead of using global publisher
ros::Publisher pub;

void chatterCallback(const shape_msgs::Mesh::ConstPtr& msg)
{
  ROS_ERROR("Mesh received");
  if (msg != nullptr)
  {
    ROS_ERROR("Mesh not nullptr");
    visualization_msgs::Marker mrkr;
    mrkr.header.frame_id = "map";
    mrkr.id = 23;
    mrkr.type = visualization_msgs::Marker::TRIANGLE_LIST;
    mrkr.action = visualization_msgs::Marker::MODIFY;
    mrkr.pose.orientation.w = 1;
    mrkr.scale.x = 1;
    mrkr.scale.y = 1;
    mrkr.scale.z = 1;

    const std::size_t len = msg->vertices.size();
    for (const shape_msgs::MeshTriangle& t : msg->triangles)
    {
      for (int i = 0; i < 3; ++i)
      {
        if (t.vertex_indices[i] < len)
        {
          mrkr.points.push_back(msg->vertices[t.vertex_indices[i]]);
        }
      }
    }

    ROS_ERROR("Publishing to RViz");
    pub.publish(mrkr);
  }
  ROS_ERROR("Ending callback");
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "translate_mesh_node");
  ros::NodeHandle nh;

  pub = nh.advertise<visualization_msgs::Marker>("mesh_marker", 5);

  ros::Subscriber sub = nh.subscribe("mesh", 5, chatterCallback);

  ros::spin();

  return 0;
}
