// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/proto/trajectory.proto

#ifndef PROTOBUF_cartographer_2fproto_2ftrajectory_2eproto__INCLUDED
#define PROTOBUF_cartographer_2fproto_2ftrajectory_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
#include "cartographer/transform/proto/transform.pb.h"
// @@protoc_insertion_point(includes)

namespace cartographer {
namespace proto {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_cartographer_2fproto_2ftrajectory_2eproto();
void protobuf_AssignDesc_cartographer_2fproto_2ftrajectory_2eproto();
void protobuf_ShutdownFile_cartographer_2fproto_2ftrajectory_2eproto();

class Trajectory;
class Trajectory_Node;

// ===================================================================

class Trajectory_Node : public ::google::protobuf::Message {
 public:
  Trajectory_Node();
  virtual ~Trajectory_Node();

  Trajectory_Node(const Trajectory_Node& from);

  inline Trajectory_Node& operator=(const Trajectory_Node& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Trajectory_Node& default_instance();

  void Swap(Trajectory_Node* other);

  // implements Message ----------------------------------------------

  Trajectory_Node* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Trajectory_Node& from);
  void MergeFrom(const Trajectory_Node& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional int64 timestamp = 1;
  inline bool has_timestamp() const;
  inline void clear_timestamp();
  static const int kTimestampFieldNumber = 1;
  inline ::google::protobuf::int64 timestamp() const;
  inline void set_timestamp(::google::protobuf::int64 value);

  // optional .cartographer.transform.proto.Rigid2d pose_2d = 2;
  inline bool has_pose_2d() const;
  inline void clear_pose_2d();
  static const int kPose2DFieldNumber = 2;
  inline const ::cartographer::transform::proto::Rigid2d& pose_2d() const;
  inline ::cartographer::transform::proto::Rigid2d* mutable_pose_2d();
  inline ::cartographer::transform::proto::Rigid2d* release_pose_2d();
  inline void set_allocated_pose_2d(::cartographer::transform::proto::Rigid2d* pose_2d);

  // optional .cartographer.transform.proto.Rigid3d pose = 5;
  inline bool has_pose() const;
  inline void clear_pose();
  static const int kPoseFieldNumber = 5;
  inline const ::cartographer::transform::proto::Rigid3d& pose() const;
  inline ::cartographer::transform::proto::Rigid3d* mutable_pose();
  inline ::cartographer::transform::proto::Rigid3d* release_pose();
  inline void set_allocated_pose(::cartographer::transform::proto::Rigid3d* pose);

  // @@protoc_insertion_point(class_scope:cartographer.proto.Trajectory.Node)
 private:
  inline void set_has_timestamp();
  inline void clear_has_timestamp();
  inline void set_has_pose_2d();
  inline void clear_has_pose_2d();
  inline void set_has_pose();
  inline void clear_has_pose();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::int64 timestamp_;
  ::cartographer::transform::proto::Rigid2d* pose_2d_;
  ::cartographer::transform::proto::Rigid3d* pose_;
  friend void  protobuf_AddDesc_cartographer_2fproto_2ftrajectory_2eproto();
  friend void protobuf_AssignDesc_cartographer_2fproto_2ftrajectory_2eproto();
  friend void protobuf_ShutdownFile_cartographer_2fproto_2ftrajectory_2eproto();

  void InitAsDefaultInstance();
  static Trajectory_Node* default_instance_;
};
// -------------------------------------------------------------------

class Trajectory : public ::google::protobuf::Message {
 public:
  Trajectory();
  virtual ~Trajectory();

  Trajectory(const Trajectory& from);

  inline Trajectory& operator=(const Trajectory& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Trajectory& default_instance();

  void Swap(Trajectory* other);

  // implements Message ----------------------------------------------

  Trajectory* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Trajectory& from);
  void MergeFrom(const Trajectory& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  typedef Trajectory_Node Node;

  // accessors -------------------------------------------------------

  // repeated .cartographer.proto.Trajectory.Node node = 1;
  inline int node_size() const;
  inline void clear_node();
  static const int kNodeFieldNumber = 1;
  inline const ::cartographer::proto::Trajectory_Node& node(int index) const;
  inline ::cartographer::proto::Trajectory_Node* mutable_node(int index);
  inline ::cartographer::proto::Trajectory_Node* add_node();
  inline const ::google::protobuf::RepeatedPtrField< ::cartographer::proto::Trajectory_Node >&
      node() const;
  inline ::google::protobuf::RepeatedPtrField< ::cartographer::proto::Trajectory_Node >*
      mutable_node();

  // @@protoc_insertion_point(class_scope:cartographer.proto.Trajectory)
 private:

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::cartographer::proto::Trajectory_Node > node_;
  friend void  protobuf_AddDesc_cartographer_2fproto_2ftrajectory_2eproto();
  friend void protobuf_AssignDesc_cartographer_2fproto_2ftrajectory_2eproto();
  friend void protobuf_ShutdownFile_cartographer_2fproto_2ftrajectory_2eproto();

  void InitAsDefaultInstance();
  static Trajectory* default_instance_;
};
// ===================================================================


// ===================================================================

// Trajectory_Node

// optional int64 timestamp = 1;
inline bool Trajectory_Node::has_timestamp() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Trajectory_Node::set_has_timestamp() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Trajectory_Node::clear_has_timestamp() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Trajectory_Node::clear_timestamp() {
  timestamp_ = GOOGLE_LONGLONG(0);
  clear_has_timestamp();
}
inline ::google::protobuf::int64 Trajectory_Node::timestamp() const {
  // @@protoc_insertion_point(field_get:cartographer.proto.Trajectory.Node.timestamp)
  return timestamp_;
}
inline void Trajectory_Node::set_timestamp(::google::protobuf::int64 value) {
  set_has_timestamp();
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:cartographer.proto.Trajectory.Node.timestamp)
}

// optional .cartographer.transform.proto.Rigid2d pose_2d = 2;
inline bool Trajectory_Node::has_pose_2d() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Trajectory_Node::set_has_pose_2d() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Trajectory_Node::clear_has_pose_2d() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Trajectory_Node::clear_pose_2d() {
  if (pose_2d_ != NULL) pose_2d_->::cartographer::transform::proto::Rigid2d::Clear();
  clear_has_pose_2d();
}
inline const ::cartographer::transform::proto::Rigid2d& Trajectory_Node::pose_2d() const {
  // @@protoc_insertion_point(field_get:cartographer.proto.Trajectory.Node.pose_2d)
  return pose_2d_ != NULL ? *pose_2d_ : *default_instance_->pose_2d_;
}
inline ::cartographer::transform::proto::Rigid2d* Trajectory_Node::mutable_pose_2d() {
  set_has_pose_2d();
  if (pose_2d_ == NULL) pose_2d_ = new ::cartographer::transform::proto::Rigid2d;
  // @@protoc_insertion_point(field_mutable:cartographer.proto.Trajectory.Node.pose_2d)
  return pose_2d_;
}
inline ::cartographer::transform::proto::Rigid2d* Trajectory_Node::release_pose_2d() {
  clear_has_pose_2d();
  ::cartographer::transform::proto::Rigid2d* temp = pose_2d_;
  pose_2d_ = NULL;
  return temp;
}
inline void Trajectory_Node::set_allocated_pose_2d(::cartographer::transform::proto::Rigid2d* pose_2d) {
  delete pose_2d_;
  pose_2d_ = pose_2d;
  if (pose_2d) {
    set_has_pose_2d();
  } else {
    clear_has_pose_2d();
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.proto.Trajectory.Node.pose_2d)
}

// optional .cartographer.transform.proto.Rigid3d pose = 5;
inline bool Trajectory_Node::has_pose() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Trajectory_Node::set_has_pose() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Trajectory_Node::clear_has_pose() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Trajectory_Node::clear_pose() {
  if (pose_ != NULL) pose_->::cartographer::transform::proto::Rigid3d::Clear();
  clear_has_pose();
}
inline const ::cartographer::transform::proto::Rigid3d& Trajectory_Node::pose() const {
  // @@protoc_insertion_point(field_get:cartographer.proto.Trajectory.Node.pose)
  return pose_ != NULL ? *pose_ : *default_instance_->pose_;
}
inline ::cartographer::transform::proto::Rigid3d* Trajectory_Node::mutable_pose() {
  set_has_pose();
  if (pose_ == NULL) pose_ = new ::cartographer::transform::proto::Rigid3d;
  // @@protoc_insertion_point(field_mutable:cartographer.proto.Trajectory.Node.pose)
  return pose_;
}
inline ::cartographer::transform::proto::Rigid3d* Trajectory_Node::release_pose() {
  clear_has_pose();
  ::cartographer::transform::proto::Rigid3d* temp = pose_;
  pose_ = NULL;
  return temp;
}
inline void Trajectory_Node::set_allocated_pose(::cartographer::transform::proto::Rigid3d* pose) {
  delete pose_;
  pose_ = pose;
  if (pose) {
    set_has_pose();
  } else {
    clear_has_pose();
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.proto.Trajectory.Node.pose)
}

// -------------------------------------------------------------------

// Trajectory

// repeated .cartographer.proto.Trajectory.Node node = 1;
inline int Trajectory::node_size() const {
  return node_.size();
}
inline void Trajectory::clear_node() {
  node_.Clear();
}
inline const ::cartographer::proto::Trajectory_Node& Trajectory::node(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.proto.Trajectory.node)
  return node_.Get(index);
}
inline ::cartographer::proto::Trajectory_Node* Trajectory::mutable_node(int index) {
  // @@protoc_insertion_point(field_mutable:cartographer.proto.Trajectory.node)
  return node_.Mutable(index);
}
inline ::cartographer::proto::Trajectory_Node* Trajectory::add_node() {
  // @@protoc_insertion_point(field_add:cartographer.proto.Trajectory.node)
  return node_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::cartographer::proto::Trajectory_Node >&
Trajectory::node() const {
  // @@protoc_insertion_point(field_list:cartographer.proto.Trajectory.node)
  return node_;
}
inline ::google::protobuf::RepeatedPtrField< ::cartographer::proto::Trajectory_Node >*
Trajectory::mutable_node() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.proto.Trajectory.node)
  return &node_;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace cartographer

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_cartographer_2fproto_2ftrajectory_2eproto__INCLUDED
