// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mimp_state.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mimp_5fstate_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mimp_5fstate_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3017000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3017003 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mimp_5fstate_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mimp_5fstate_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mimp_5fstate_2eproto;
class MimpState;
struct MimpStateDefaultTypeInternal;
extern MimpStateDefaultTypeInternal _MimpState_default_instance_;
PROTOBUF_NAMESPACE_OPEN
template<> ::MimpState* Arena::CreateMaybeMessage<::MimpState>(Arena*);
PROTOBUF_NAMESPACE_CLOSE

// ===================================================================

class MimpState final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:MimpState) */ {
 public:
  inline MimpState() : MimpState(nullptr) {}
  ~MimpState() override;
  explicit constexpr MimpState(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  MimpState(const MimpState& from);
  MimpState(MimpState&& from) noexcept
    : MimpState() {
    *this = ::std::move(from);
  }

  inline MimpState& operator=(const MimpState& from) {
    CopyFrom(from);
    return *this;
  }
  inline MimpState& operator=(MimpState&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const MimpState& default_instance() {
    return *internal_default_instance();
  }
  static inline const MimpState* internal_default_instance() {
    return reinterpret_cast<const MimpState*>(
               &_MimpState_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MimpState& a, MimpState& b) {
    a.Swap(&b);
  }
  inline void Swap(MimpState* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(MimpState* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MimpState* New() const final {
    return new MimpState();
  }

  MimpState* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MimpState>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const MimpState& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const MimpState& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message*to, const ::PROTOBUF_NAMESPACE_ID::Message&from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(MimpState* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "MimpState";
  }
  protected:
  explicit MimpState(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kToolTrafoFieldNumber = 1,
    kClosestPointFieldNumber = 2,
    kCoordinatesFieldNumber = 3,
    kTriangleIdFieldNumber = 4,
  };
  // repeated double tool_trafo = 1;
  int tool_trafo_size() const;
  private:
  int _internal_tool_trafo_size() const;
  public:
  void clear_tool_trafo();
  private:
  double _internal_tool_trafo(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_tool_trafo() const;
  void _internal_add_tool_trafo(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_tool_trafo();
  public:
  double tool_trafo(int index) const;
  void set_tool_trafo(int index, double value);
  void add_tool_trafo(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      tool_trafo() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_tool_trafo();

  // repeated double closest_point = 2;
  int closest_point_size() const;
  private:
  int _internal_closest_point_size() const;
  public:
  void clear_closest_point();
  private:
  double _internal_closest_point(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_closest_point() const;
  void _internal_add_closest_point(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_closest_point();
  public:
  double closest_point(int index) const;
  void set_closest_point(int index, double value);
  void add_closest_point(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      closest_point() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_closest_point();

  // repeated double coordinates = 3;
  int coordinates_size() const;
  private:
  int _internal_coordinates_size() const;
  public:
  void clear_coordinates();
  private:
  double _internal_coordinates(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_coordinates() const;
  void _internal_add_coordinates(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_coordinates();
  public:
  double coordinates(int index) const;
  void set_coordinates(int index, double value);
  void add_coordinates(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      coordinates() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_coordinates();

  // int64 triangle_id = 4;
  void clear_triangle_id();
  ::PROTOBUF_NAMESPACE_ID::int64 triangle_id() const;
  void set_triangle_id(::PROTOBUF_NAMESPACE_ID::int64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int64 _internal_triangle_id() const;
  void _internal_set_triangle_id(::PROTOBUF_NAMESPACE_ID::int64 value);
  public:

  // @@protoc_insertion_point(class_scope:MimpState)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > tool_trafo_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > closest_point_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > coordinates_;
  ::PROTOBUF_NAMESPACE_ID::int64 triangle_id_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_mimp_5fstate_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MimpState

// repeated double tool_trafo = 1;
inline int MimpState::_internal_tool_trafo_size() const {
  return tool_trafo_.size();
}
inline int MimpState::tool_trafo_size() const {
  return _internal_tool_trafo_size();
}
inline void MimpState::clear_tool_trafo() {
  tool_trafo_.Clear();
}
inline double MimpState::_internal_tool_trafo(int index) const {
  return tool_trafo_.Get(index);
}
inline double MimpState::tool_trafo(int index) const {
  // @@protoc_insertion_point(field_get:MimpState.tool_trafo)
  return _internal_tool_trafo(index);
}
inline void MimpState::set_tool_trafo(int index, double value) {
  tool_trafo_.Set(index, value);
  // @@protoc_insertion_point(field_set:MimpState.tool_trafo)
}
inline void MimpState::_internal_add_tool_trafo(double value) {
  tool_trafo_.Add(value);
}
inline void MimpState::add_tool_trafo(double value) {
  _internal_add_tool_trafo(value);
  // @@protoc_insertion_point(field_add:MimpState.tool_trafo)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
MimpState::_internal_tool_trafo() const {
  return tool_trafo_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
MimpState::tool_trafo() const {
  // @@protoc_insertion_point(field_list:MimpState.tool_trafo)
  return _internal_tool_trafo();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
MimpState::_internal_mutable_tool_trafo() {
  return &tool_trafo_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
MimpState::mutable_tool_trafo() {
  // @@protoc_insertion_point(field_mutable_list:MimpState.tool_trafo)
  return _internal_mutable_tool_trafo();
}

// repeated double closest_point = 2;
inline int MimpState::_internal_closest_point_size() const {
  return closest_point_.size();
}
inline int MimpState::closest_point_size() const {
  return _internal_closest_point_size();
}
inline void MimpState::clear_closest_point() {
  closest_point_.Clear();
}
inline double MimpState::_internal_closest_point(int index) const {
  return closest_point_.Get(index);
}
inline double MimpState::closest_point(int index) const {
  // @@protoc_insertion_point(field_get:MimpState.closest_point)
  return _internal_closest_point(index);
}
inline void MimpState::set_closest_point(int index, double value) {
  closest_point_.Set(index, value);
  // @@protoc_insertion_point(field_set:MimpState.closest_point)
}
inline void MimpState::_internal_add_closest_point(double value) {
  closest_point_.Add(value);
}
inline void MimpState::add_closest_point(double value) {
  _internal_add_closest_point(value);
  // @@protoc_insertion_point(field_add:MimpState.closest_point)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
MimpState::_internal_closest_point() const {
  return closest_point_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
MimpState::closest_point() const {
  // @@protoc_insertion_point(field_list:MimpState.closest_point)
  return _internal_closest_point();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
MimpState::_internal_mutable_closest_point() {
  return &closest_point_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
MimpState::mutable_closest_point() {
  // @@protoc_insertion_point(field_mutable_list:MimpState.closest_point)
  return _internal_mutable_closest_point();
}

// repeated double coordinates = 3;
inline int MimpState::_internal_coordinates_size() const {
  return coordinates_.size();
}
inline int MimpState::coordinates_size() const {
  return _internal_coordinates_size();
}
inline void MimpState::clear_coordinates() {
  coordinates_.Clear();
}
inline double MimpState::_internal_coordinates(int index) const {
  return coordinates_.Get(index);
}
inline double MimpState::coordinates(int index) const {
  // @@protoc_insertion_point(field_get:MimpState.coordinates)
  return _internal_coordinates(index);
}
inline void MimpState::set_coordinates(int index, double value) {
  coordinates_.Set(index, value);
  // @@protoc_insertion_point(field_set:MimpState.coordinates)
}
inline void MimpState::_internal_add_coordinates(double value) {
  coordinates_.Add(value);
}
inline void MimpState::add_coordinates(double value) {
  _internal_add_coordinates(value);
  // @@protoc_insertion_point(field_add:MimpState.coordinates)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
MimpState::_internal_coordinates() const {
  return coordinates_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
MimpState::coordinates() const {
  // @@protoc_insertion_point(field_list:MimpState.coordinates)
  return _internal_coordinates();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
MimpState::_internal_mutable_coordinates() {
  return &coordinates_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
MimpState::mutable_coordinates() {
  // @@protoc_insertion_point(field_mutable_list:MimpState.coordinates)
  return _internal_mutable_coordinates();
}

// int64 triangle_id = 4;
inline void MimpState::clear_triangle_id() {
  triangle_id_ = int64_t{0};
}
inline ::PROTOBUF_NAMESPACE_ID::int64 MimpState::_internal_triangle_id() const {
  return triangle_id_;
}
inline ::PROTOBUF_NAMESPACE_ID::int64 MimpState::triangle_id() const {
  // @@protoc_insertion_point(field_get:MimpState.triangle_id)
  return _internal_triangle_id();
}
inline void MimpState::_internal_set_triangle_id(::PROTOBUF_NAMESPACE_ID::int64 value) {
  
  triangle_id_ = value;
}
inline void MimpState::set_triangle_id(::PROTOBUF_NAMESPACE_ID::int64 value) {
  _internal_set_triangle_id(value);
  // @@protoc_insertion_point(field_set:MimpState.triangle_id)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)


// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mimp_5fstate_2eproto
