// Generated by gencpp from file quadrotor_msgs/TakeoffLand.msg
// DO NOT EDIT!


#ifndef QUADROTOR_MSGS_MESSAGE_TAKEOFFLAND_H
#define QUADROTOR_MSGS_MESSAGE_TAKEOFFLAND_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct TakeoffLand_
{
  typedef TakeoffLand_<ContainerAllocator> Type;

  TakeoffLand_()
    : takeoff_land_cmd(0)  {
    }
  TakeoffLand_(const ContainerAllocator& _alloc)
    : takeoff_land_cmd(0)  {
  (void)_alloc;
    }



   typedef uint8_t _takeoff_land_cmd_type;
  _takeoff_land_cmd_type takeoff_land_cmd;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(TAKEOFF)
  #undef TAKEOFF
#endif
#if defined(_WIN32) && defined(LAND)
  #undef LAND
#endif

  enum {
    TAKEOFF = 1u,
    LAND = 2u,
  };


  typedef boost::shared_ptr< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> const> ConstPtr;

}; // struct TakeoffLand_

typedef ::quadrotor_msgs::TakeoffLand_<std::allocator<void> > TakeoffLand;

typedef boost::shared_ptr< ::quadrotor_msgs::TakeoffLand > TakeoffLandPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::TakeoffLand const> TakeoffLandConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::quadrotor_msgs::TakeoffLand_<ContainerAllocator1> & lhs, const ::quadrotor_msgs::TakeoffLand_<ContainerAllocator2> & rhs)
{
  return lhs.takeoff_land_cmd == rhs.takeoff_land_cmd;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::quadrotor_msgs::TakeoffLand_<ContainerAllocator1> & lhs, const ::quadrotor_msgs::TakeoffLand_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b30b3b93263aae01746755d3b8ff620f";
  }

  static const char* value(const ::quadrotor_msgs::TakeoffLand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb30b3b93263aae01ULL;
  static const uint64_t static_value2 = 0x746755d3b8ff620fULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quadrotor_msgs/TakeoffLand";
  }

  static const char* value(const ::quadrotor_msgs::TakeoffLand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 TAKEOFF = 1\n"
"uint8 LAND = 2\n"
"uint8 takeoff_land_cmd\n"
;
  }

  static const char* value(const ::quadrotor_msgs::TakeoffLand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.takeoff_land_cmd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TakeoffLand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::TakeoffLand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::quadrotor_msgs::TakeoffLand_<ContainerAllocator>& v)
  {
    s << indent << "takeoff_land_cmd: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.takeoff_land_cmd);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_TAKEOFFLAND_H
