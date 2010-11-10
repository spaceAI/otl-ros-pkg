#ifndef EGEOMETRY_E2MSG_H
#define EGEOMETRY_E2MSG_H

#include <egeometry/coordinates.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

namespace egeometry
{
  /// convert coordinates to Transform msg
  /// @param [in] coords input coordinates
  /// @param [out] tf output transform message
  inline void CoordinatesToTFMsg(const Coordinates& coords, geometry_msgs::TransformStamped& tf)
  {
    tf.header.frame_id = coords.GetName();
    tf.child_frame_id = coords.GetParent().GetName();

    tf.transform.translation.x = coords.GetWorldPosition().GetX();
    tf.transform.translation.y = coords.GetWorldPosition().GetY();
    tf.transform.translation.z = coords.GetWorldPosition().GetZ();
    
    double q[4];
    coords.GetQuaternion(q);
    tf.transform.rotation.x = q[0];
    tf.transform.rotation.y = q[1];
    tf.transform.rotation.z = q[2];
    tf.transform.rotation.w = q[3];
  }

  /// convert coordinates to Transform msg
  /// @param [in] coords input coordinates
  /// @param [out] tf output transform message
  inline void TFMsgToCoordinates(const geometry_msgs::TransformStamped& tf, Coordinates& coords)
  {
    coords.SetName(tf.header.frame_id);
    //tf.child_frame_id = coords.GetParent().GetName();
    
    coords.SetPosition(FloatVector(tf.transform.translation.x,
				   tf.transform.translation.y,
				   tf.transform.translation.z));
    
    double q[4];
    q[0] = tf.transform.rotation.x;
    q[1] = tf.transform.rotation.y;
    q[2] = tf.transform.rotation.z;
    q[3] = tf.transform.rotation.w;
    coords.SetQuaternion(q);
  }

  /// convert coordinates to Pose msg
  /// @param [in] coords input coordinates
  /// @param [out] pose output Pose message
  inline void CoordinatesToPoseMsg(const Coordinates& coords, geometry_msgs::PoseStamped& pose)
  {
    pose.header.frame_id = coords.GetName();
    
    pose.pose.position.x = coords.GetWorldPosition().GetX();
    pose.pose.position.y = coords.GetWorldPosition().GetY();
    pose.pose.position.z = coords.GetWorldPosition().GetZ();
    
    double q[4];
    coords.GetQuaternion(q);
    pose.pose.orientation.x = q[0];
    pose.pose.orientation.y = q[1];
    pose.pose.orientation.z = q[2];
    pose.pose.orientation.w = q[3];
  }

  /// convert coordinates to Pose msg
  /// @param [in] coords input coordinates
  /// @param [out] pose output Pose message
  inline void PoseMsgToCoordinates(const geometry_msgs::PoseStamped& pose, Coordinates& coords)
  {
    coords.SetName(pose.header.frame_id);
    
    coords.SetPosition(FloatVector(pose.pose.position.x, 
				   pose.pose.position.y,
				   pose.pose.position.z));
    
    double q[4];
    q[0] = pose.pose.orientation.x;
    q[1] = pose.pose.orientation.y;
    q[2] = pose.pose.orientation.z;
    q[3] = pose.pose.orientation.w;
    coords.SetQuaternion(q);
  }

  /// convert Rotation to Quaternion msg
  /// @param [in] rot input matrix
  /// @param [out] msg output quaternion message
 inline void RotationToQuaternionMsg(const FloatMatrix& rot, geometry_msgs::Quaternion& msg)
  {
    double q[4];
    rot.GetQuaternion(q);
    msg.x = q[0];
    msg.y = q[1];
    msg.z = q[2];
    msg.w = q[3];
  }

  /// convert Rotation to Quaternion msg
  /// @param [in] rot input matrix
  /// @param [out] msg output quaternion message
 inline void QuaternionMsgToRotation(const geometry_msgs::Quaternion& msg, FloatMatrix& rot)
  {
    double q[4];
    q[0] = msg.x;
    q[1] = msg.y;
    q[2] = msg.z;
    q[3] = msg.w;
    rot.SetQuaternion(q);
  }

  /// convert Position to Point msg
  /// @param [in] pos input vector
  /// @param [out] msg output Point message
  inline void PositionToPointMsg(const FloatVector& pos, geometry_msgs::Point& msg)
  {
    msg.x = pos.GetX();
    msg.y = pos.GetY();
    msg.z = pos.GetZ();
  }

  /// convert Position to Point msg
  /// @param [in] pos input vector
  /// @param [out] msg output Point message
  inline void PointMsgToPosition(const geometry_msgs::Point& msg, FloatVector& pos)
  {
    pos.SetX(msg.x);
    pos.SetY(msg.y);
    pos.SetZ(msg.z);
  }
}

#endif //EGEOMETRY_E2MSG_H
