// Generated by gencpp from file serial_ros/PoseMsgs.msg
// DO NOT EDIT!


#ifndef SERIAL_ROS_MESSAGE_POSEMSGS_H
#define SERIAL_ROS_MESSAGE_POSEMSGS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace serial_ros
{
template <class ContainerAllocator>
struct PoseMsgs_
{
  typedef PoseMsgs_<ContainerAllocator> Type;

  PoseMsgs_()
    : poseMsgs()
    , carId()  {
    }
  PoseMsgs_(const ContainerAllocator& _alloc)
    : poseMsgs(_alloc)
    , carId(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator> >::other >  _poseMsgs_type;
  _poseMsgs_type poseMsgs;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _carId_type;
  _carId_type carId;





  typedef boost::shared_ptr< ::serial_ros::PoseMsgs_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serial_ros::PoseMsgs_<ContainerAllocator> const> ConstPtr;

}; // struct PoseMsgs_

typedef ::serial_ros::PoseMsgs_<std::allocator<void> > PoseMsgs;

typedef boost::shared_ptr< ::serial_ros::PoseMsgs > PoseMsgsPtr;
typedef boost::shared_ptr< ::serial_ros::PoseMsgs const> PoseMsgsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::serial_ros::PoseMsgs_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::serial_ros::PoseMsgs_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::serial_ros::PoseMsgs_<ContainerAllocator1> & lhs, const ::serial_ros::PoseMsgs_<ContainerAllocator2> & rhs)
{
  return lhs.poseMsgs == rhs.poseMsgs &&
    lhs.carId == rhs.carId;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::serial_ros::PoseMsgs_<ContainerAllocator1> & lhs, const ::serial_ros::PoseMsgs_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace serial_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::serial_ros::PoseMsgs_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serial_ros::PoseMsgs_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_ros::PoseMsgs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_ros::PoseMsgs_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_ros::PoseMsgs_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_ros::PoseMsgs_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::serial_ros::PoseMsgs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8c76c3222856d20930581a2a503a432d";
  }

  static const char* value(const ::serial_ros::PoseMsgs_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8c76c3222856d209ULL;
  static const uint64_t static_value2 = 0x30581a2a503a432dULL;
};

template<class ContainerAllocator>
struct DataType< ::serial_ros::PoseMsgs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "serial_ros/PoseMsgs";
  }

  static const char* value(const ::serial_ros::PoseMsgs_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::serial_ros::PoseMsgs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/PoseWithCovarianceStamped[] poseMsgs\n"
"string carId\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseWithCovarianceStamped\n"
"# This expresses an estimated pose with a reference coordinate frame and timestamp\n"
"\n"
"Header header\n"
"PoseWithCovariance pose\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseWithCovariance\n"
"# This represents a pose in free space with uncertainty.\n"
"\n"
"Pose pose\n"
"\n"
"# Row-major representation of the 6x6 covariance matrix\n"
"# The orientation parameters use a fixed-axis representation.\n"
"# In order, the parameters are:\n"
"# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n"
"float64[36] covariance\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::serial_ros::PoseMsgs_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::serial_ros::PoseMsgs_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.poseMsgs);
      stream.next(m.carId);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PoseMsgs_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::serial_ros::PoseMsgs_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::serial_ros::PoseMsgs_<ContainerAllocator>& v)
  {
    s << indent << "poseMsgs[]" << std::endl;
    for (size_t i = 0; i < v.poseMsgs.size(); ++i)
    {
      s << indent << "  poseMsgs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.poseMsgs[i]);
    }
    s << indent << "carId: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.carId);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERIAL_ROS_MESSAGE_POSEMSGS_H
