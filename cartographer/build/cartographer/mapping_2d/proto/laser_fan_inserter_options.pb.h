// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping_2d/proto/laser_fan_inserter_options.proto

#ifndef PROTOBUF_cartographer_2fmapping_5f2d_2fproto_2flaser_5ffan_5finserter_5foptions_2eproto__INCLUDED
#define PROTOBUF_cartographer_2fmapping_5f2d_2fproto_2flaser_5ffan_5finserter_5foptions_2eproto__INCLUDED

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
// @@protoc_insertion_point(includes)

namespace cartographer {
namespace mapping_2d {
namespace proto {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_cartographer_2fmapping_5f2d_2fproto_2flaser_5ffan_5finserter_5foptions_2eproto();
void protobuf_AssignDesc_cartographer_2fmapping_5f2d_2fproto_2flaser_5ffan_5finserter_5foptions_2eproto();
void protobuf_ShutdownFile_cartographer_2fmapping_5f2d_2fproto_2flaser_5ffan_5finserter_5foptions_2eproto();

class LaserFanInserterOptions;

// ===================================================================

class LaserFanInserterOptions : public ::google::protobuf::Message {
 public:
  LaserFanInserterOptions();
  virtual ~LaserFanInserterOptions();

  LaserFanInserterOptions(const LaserFanInserterOptions& from);

  inline LaserFanInserterOptions& operator=(const LaserFanInserterOptions& from) {
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
  static const LaserFanInserterOptions& default_instance();

  void Swap(LaserFanInserterOptions* other);

  // implements Message ----------------------------------------------

  LaserFanInserterOptions* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const LaserFanInserterOptions& from);
  void MergeFrom(const LaserFanInserterOptions& from);
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

  // optional double hit_probability = 1;
  inline bool has_hit_probability() const;
  inline void clear_hit_probability();
  static const int kHitProbabilityFieldNumber = 1;
  inline double hit_probability() const;
  inline void set_hit_probability(double value);

  // optional double miss_probability = 2;
  inline bool has_miss_probability() const;
  inline void clear_miss_probability();
  static const int kMissProbabilityFieldNumber = 2;
  inline double miss_probability() const;
  inline void set_miss_probability(double value);

  // optional bool insert_free_space = 3;
  inline bool has_insert_free_space() const;
  inline void clear_insert_free_space();
  static const int kInsertFreeSpaceFieldNumber = 3;
  inline bool insert_free_space() const;
  inline void set_insert_free_space(bool value);

  // @@protoc_insertion_point(class_scope:cartographer.mapping_2d.proto.LaserFanInserterOptions)
 private:
  inline void set_has_hit_probability();
  inline void clear_has_hit_probability();
  inline void set_has_miss_probability();
  inline void clear_has_miss_probability();
  inline void set_has_insert_free_space();
  inline void clear_has_insert_free_space();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double hit_probability_;
  double miss_probability_;
  bool insert_free_space_;
  friend void  protobuf_AddDesc_cartographer_2fmapping_5f2d_2fproto_2flaser_5ffan_5finserter_5foptions_2eproto();
  friend void protobuf_AssignDesc_cartographer_2fmapping_5f2d_2fproto_2flaser_5ffan_5finserter_5foptions_2eproto();
  friend void protobuf_ShutdownFile_cartographer_2fmapping_5f2d_2fproto_2flaser_5ffan_5finserter_5foptions_2eproto();

  void InitAsDefaultInstance();
  static LaserFanInserterOptions* default_instance_;
};
// ===================================================================


// ===================================================================

// LaserFanInserterOptions

// optional double hit_probability = 1;
inline bool LaserFanInserterOptions::has_hit_probability() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void LaserFanInserterOptions::set_has_hit_probability() {
  _has_bits_[0] |= 0x00000001u;
}
inline void LaserFanInserterOptions::clear_has_hit_probability() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void LaserFanInserterOptions::clear_hit_probability() {
  hit_probability_ = 0;
  clear_has_hit_probability();
}
inline double LaserFanInserterOptions::hit_probability() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_2d.proto.LaserFanInserterOptions.hit_probability)
  return hit_probability_;
}
inline void LaserFanInserterOptions::set_hit_probability(double value) {
  set_has_hit_probability();
  hit_probability_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_2d.proto.LaserFanInserterOptions.hit_probability)
}

// optional double miss_probability = 2;
inline bool LaserFanInserterOptions::has_miss_probability() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void LaserFanInserterOptions::set_has_miss_probability() {
  _has_bits_[0] |= 0x00000002u;
}
inline void LaserFanInserterOptions::clear_has_miss_probability() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void LaserFanInserterOptions::clear_miss_probability() {
  miss_probability_ = 0;
  clear_has_miss_probability();
}
inline double LaserFanInserterOptions::miss_probability() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_2d.proto.LaserFanInserterOptions.miss_probability)
  return miss_probability_;
}
inline void LaserFanInserterOptions::set_miss_probability(double value) {
  set_has_miss_probability();
  miss_probability_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_2d.proto.LaserFanInserterOptions.miss_probability)
}

// optional bool insert_free_space = 3;
inline bool LaserFanInserterOptions::has_insert_free_space() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void LaserFanInserterOptions::set_has_insert_free_space() {
  _has_bits_[0] |= 0x00000004u;
}
inline void LaserFanInserterOptions::clear_has_insert_free_space() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void LaserFanInserterOptions::clear_insert_free_space() {
  insert_free_space_ = false;
  clear_has_insert_free_space();
}
inline bool LaserFanInserterOptions::insert_free_space() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_2d.proto.LaserFanInserterOptions.insert_free_space)
  return insert_free_space_;
}
inline void LaserFanInserterOptions::set_insert_free_space(bool value) {
  set_has_insert_free_space();
  insert_free_space_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_2d.proto.LaserFanInserterOptions.insert_free_space)
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace mapping_2d
}  // namespace cartographer

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_cartographer_2fmapping_5f2d_2fproto_2flaser_5ffan_5finserter_5foptions_2eproto__INCLUDED
