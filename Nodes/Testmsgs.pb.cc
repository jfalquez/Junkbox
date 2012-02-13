// Generated by the protocol buffer compiler.  DO NOT EDIT!

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "Testmsgs.pb.h"
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>

namespace {

const ::google::protobuf::Descriptor* Professor_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Professor_reflection_ = NULL;
const ::google::protobuf::Descriptor* Student_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Student_reflection_ = NULL;
const ::google::protobuf::Descriptor* Staff_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Staff_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_Testmsgs_2eproto() {
  protobuf_AddDesc_Testmsgs_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "Testmsgs.proto");
  GOOGLE_CHECK(file != NULL);
  Professor_descriptor_ = file->message_type(0);
  static const int Professor_offsets_[3] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Professor, name_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Professor, id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Professor, email_),
  };
  Professor_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Professor_descriptor_,
      Professor::default_instance_,
      Professor_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Professor, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Professor, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Professor));
  Student_descriptor_ = file->message_type(1);
  static const int Student_offsets_[3] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Student, name_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Student, id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Student, email_),
  };
  Student_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Student_descriptor_,
      Student::default_instance_,
      Student_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Student, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Student, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Student));
  Staff_descriptor_ = file->message_type(2);
  static const int Staff_offsets_[3] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Staff, name_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Staff, id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Staff, email_),
  };
  Staff_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Staff_descriptor_,
      Staff::default_instance_,
      Staff_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Staff, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Staff, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Staff));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_Testmsgs_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Professor_descriptor_, &Professor::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Student_descriptor_, &Student::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Staff_descriptor_, &Staff::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_Testmsgs_2eproto() {
  delete Professor::default_instance_;
  delete Professor_reflection_;
  delete Student::default_instance_;
  delete Student_reflection_;
  delete Staff::default_instance_;
  delete Staff_reflection_;
}

void protobuf_AddDesc_Testmsgs_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\016Testmsgs.proto\"4\n\tProfessor\022\014\n\004name\030\001 "
    "\002(\t\022\n\n\002id\030\002 \002(\005\022\r\n\005email\030\003 \001(\t\"2\n\007Studen"
    "t\022\014\n\004name\030\001 \002(\t\022\n\n\002id\030\002 \002(\005\022\r\n\005email\030\003 \001"
    "(\t\"0\n\005Staff\022\014\n\004name\030\001 \002(\t\022\n\n\002id\030\002 \002(\005\022\r\n"
    "\005email\030\003 \001(\t", 172);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "Testmsgs.proto", &protobuf_RegisterTypes);
  Professor::default_instance_ = new Professor();
  Student::default_instance_ = new Student();
  Staff::default_instance_ = new Staff();
  Professor::default_instance_->InitAsDefaultInstance();
  Student::default_instance_->InitAsDefaultInstance();
  Staff::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_Testmsgs_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_Testmsgs_2eproto {
  StaticDescriptorInitializer_Testmsgs_2eproto() {
    protobuf_AddDesc_Testmsgs_2eproto();
  }
} static_descriptor_initializer_Testmsgs_2eproto_;


// ===================================================================

const ::std::string Professor::_default_name_;
const ::std::string Professor::_default_email_;
#ifndef _MSC_VER
const int Professor::kNameFieldNumber;
const int Professor::kIdFieldNumber;
const int Professor::kEmailFieldNumber;
#endif  // !_MSC_VER

Professor::Professor() {
  SharedCtor();
}

void Professor::InitAsDefaultInstance() {
}

Professor::Professor(const Professor& from) {
  SharedCtor();
  MergeFrom(from);
}

void Professor::SharedCtor() {
  _cached_size_ = 0;
  name_ = const_cast< ::std::string*>(&_default_name_);
  id_ = 0;
  email_ = const_cast< ::std::string*>(&_default_email_);
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Professor::~Professor() {
  SharedDtor();
}

void Professor::SharedDtor() {
  if (name_ != &_default_name_) {
    delete name_;
  }
  if (email_ != &_default_email_) {
    delete email_;
  }
  if (this != default_instance_) {
  }
}

const ::google::protobuf::Descriptor* Professor::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Professor_descriptor_;
}

const Professor& Professor::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_Testmsgs_2eproto();  return *default_instance_;
}

Professor* Professor::default_instance_ = NULL;

Professor* Professor::New() const {
  return new Professor;
}

void Professor::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (_has_bit(0)) {
      if (name_ != &_default_name_) {
        name_->clear();
      }
    }
    id_ = 0;
    if (_has_bit(2)) {
      if (email_ != &_default_email_) {
        email_->clear();
      }
    }
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Professor::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string name = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) !=
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
          goto handle_uninterpreted;
        }
        DO_(::google::protobuf::internal::WireFormatLite::ReadString(
              input, this->mutable_name()));
        ::google::protobuf::internal::WireFormat::VerifyUTF8String(
          this->name().data(), this->name().length(),
          ::google::protobuf::internal::WireFormat::PARSE);
        if (input->ExpectTag(16)) goto parse_id;
        break;
      }
      
      // required int32 id = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) !=
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
          goto handle_uninterpreted;
        }
       parse_id:
        DO_(::google::protobuf::internal::WireFormatLite::ReadInt32(
              input, &id_));
        _set_bit(1);
        if (input->ExpectTag(26)) goto parse_email;
        break;
      }
      
      // optional string email = 3;
      case 3: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) !=
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
          goto handle_uninterpreted;
        }
       parse_email:
        DO_(::google::protobuf::internal::WireFormatLite::ReadString(
              input, this->mutable_email()));
        ::google::protobuf::internal::WireFormat::VerifyUTF8String(
          this->email().data(), this->email().length(),
          ::google::protobuf::internal::WireFormat::PARSE);
        if (input->ExpectAtEnd()) return true;
        break;
      }
      
      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void Professor::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  ::google::protobuf::uint8* raw_buffer = output->GetDirectBufferForNBytesAndAdvance(_cached_size_);
  if (raw_buffer != NULL) {
    Professor::SerializeWithCachedSizesToArray(raw_buffer);
    return;
  }
  
  // required string name = 1;
  if (_has_bit(0)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    ::google::protobuf::internal::WireFormatLite::WriteString(
      1, this->name(), output);
  }
  
  // required int32 id = 2;
  if (_has_bit(1)) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(2, this->id(), output);
  }
  
  // optional string email = 3;
  if (_has_bit(2)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->email().data(), this->email().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    ::google::protobuf::internal::WireFormatLite::WriteString(
      3, this->email(), output);
  }
  
  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* Professor::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // required string name = 1;
  if (_has_bit(0)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->name(), target);
  }
  
  // required int32 id = 2;
  if (_has_bit(1)) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(2, this->id(), target);
  }
  
  // optional string email = 3;
  if (_has_bit(2)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->email().data(), this->email().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        3, this->email(), target);
  }
  
  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int Professor::ByteSize() const {
  int total_size = 0;
  
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required string name = 1;
    if (has_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->name());
    }
    
    // required int32 id = 2;
    if (has_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->id());
    }
    
    // optional string email = 3;
    if (has_email()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->email());
    }
    
  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  _cached_size_ = total_size;
  return total_size;
}

void Professor::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Professor* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Professor*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Professor::MergeFrom(const Professor& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from._has_bit(0)) {
      set_name(from.name());
    }
    if (from._has_bit(1)) {
      set_id(from.id());
    }
    if (from._has_bit(2)) {
      set_email(from.email());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Professor::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Professor::CopyFrom(const Professor& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Professor::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;
  
  return true;
}

void Professor::Swap(Professor* other) {
  if (other != this) {
    std::swap(name_, other->name_);
    std::swap(id_, other->id_);
    std::swap(email_, other->email_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Professor::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Professor_descriptor_;
  metadata.reflection = Professor_reflection_;
  return metadata;
}


// ===================================================================

const ::std::string Student::_default_name_;
const ::std::string Student::_default_email_;
#ifndef _MSC_VER
const int Student::kNameFieldNumber;
const int Student::kIdFieldNumber;
const int Student::kEmailFieldNumber;
#endif  // !_MSC_VER

Student::Student() {
  SharedCtor();
}

void Student::InitAsDefaultInstance() {
}

Student::Student(const Student& from) {
  SharedCtor();
  MergeFrom(from);
}

void Student::SharedCtor() {
  _cached_size_ = 0;
  name_ = const_cast< ::std::string*>(&_default_name_);
  id_ = 0;
  email_ = const_cast< ::std::string*>(&_default_email_);
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Student::~Student() {
  SharedDtor();
}

void Student::SharedDtor() {
  if (name_ != &_default_name_) {
    delete name_;
  }
  if (email_ != &_default_email_) {
    delete email_;
  }
  if (this != default_instance_) {
  }
}

const ::google::protobuf::Descriptor* Student::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Student_descriptor_;
}

const Student& Student::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_Testmsgs_2eproto();  return *default_instance_;
}

Student* Student::default_instance_ = NULL;

Student* Student::New() const {
  return new Student;
}

void Student::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (_has_bit(0)) {
      if (name_ != &_default_name_) {
        name_->clear();
      }
    }
    id_ = 0;
    if (_has_bit(2)) {
      if (email_ != &_default_email_) {
        email_->clear();
      }
    }
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Student::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string name = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) !=
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
          goto handle_uninterpreted;
        }
        DO_(::google::protobuf::internal::WireFormatLite::ReadString(
              input, this->mutable_name()));
        ::google::protobuf::internal::WireFormat::VerifyUTF8String(
          this->name().data(), this->name().length(),
          ::google::protobuf::internal::WireFormat::PARSE);
        if (input->ExpectTag(16)) goto parse_id;
        break;
      }
      
      // required int32 id = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) !=
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
          goto handle_uninterpreted;
        }
       parse_id:
        DO_(::google::protobuf::internal::WireFormatLite::ReadInt32(
              input, &id_));
        _set_bit(1);
        if (input->ExpectTag(26)) goto parse_email;
        break;
      }
      
      // optional string email = 3;
      case 3: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) !=
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
          goto handle_uninterpreted;
        }
       parse_email:
        DO_(::google::protobuf::internal::WireFormatLite::ReadString(
              input, this->mutable_email()));
        ::google::protobuf::internal::WireFormat::VerifyUTF8String(
          this->email().data(), this->email().length(),
          ::google::protobuf::internal::WireFormat::PARSE);
        if (input->ExpectAtEnd()) return true;
        break;
      }
      
      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void Student::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  ::google::protobuf::uint8* raw_buffer = output->GetDirectBufferForNBytesAndAdvance(_cached_size_);
  if (raw_buffer != NULL) {
    Student::SerializeWithCachedSizesToArray(raw_buffer);
    return;
  }
  
  // required string name = 1;
  if (_has_bit(0)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    ::google::protobuf::internal::WireFormatLite::WriteString(
      1, this->name(), output);
  }
  
  // required int32 id = 2;
  if (_has_bit(1)) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(2, this->id(), output);
  }
  
  // optional string email = 3;
  if (_has_bit(2)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->email().data(), this->email().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    ::google::protobuf::internal::WireFormatLite::WriteString(
      3, this->email(), output);
  }
  
  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* Student::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // required string name = 1;
  if (_has_bit(0)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->name(), target);
  }
  
  // required int32 id = 2;
  if (_has_bit(1)) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(2, this->id(), target);
  }
  
  // optional string email = 3;
  if (_has_bit(2)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->email().data(), this->email().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        3, this->email(), target);
  }
  
  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int Student::ByteSize() const {
  int total_size = 0;
  
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required string name = 1;
    if (has_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->name());
    }
    
    // required int32 id = 2;
    if (has_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->id());
    }
    
    // optional string email = 3;
    if (has_email()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->email());
    }
    
  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  _cached_size_ = total_size;
  return total_size;
}

void Student::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Student* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Student*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Student::MergeFrom(const Student& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from._has_bit(0)) {
      set_name(from.name());
    }
    if (from._has_bit(1)) {
      set_id(from.id());
    }
    if (from._has_bit(2)) {
      set_email(from.email());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Student::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Student::CopyFrom(const Student& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Student::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;
  
  return true;
}

void Student::Swap(Student* other) {
  if (other != this) {
    std::swap(name_, other->name_);
    std::swap(id_, other->id_);
    std::swap(email_, other->email_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Student::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Student_descriptor_;
  metadata.reflection = Student_reflection_;
  return metadata;
}


// ===================================================================

const ::std::string Staff::_default_name_;
const ::std::string Staff::_default_email_;
#ifndef _MSC_VER
const int Staff::kNameFieldNumber;
const int Staff::kIdFieldNumber;
const int Staff::kEmailFieldNumber;
#endif  // !_MSC_VER

Staff::Staff() {
  SharedCtor();
}

void Staff::InitAsDefaultInstance() {
}

Staff::Staff(const Staff& from) {
  SharedCtor();
  MergeFrom(from);
}

void Staff::SharedCtor() {
  _cached_size_ = 0;
  name_ = const_cast< ::std::string*>(&_default_name_);
  id_ = 0;
  email_ = const_cast< ::std::string*>(&_default_email_);
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Staff::~Staff() {
  SharedDtor();
}

void Staff::SharedDtor() {
  if (name_ != &_default_name_) {
    delete name_;
  }
  if (email_ != &_default_email_) {
    delete email_;
  }
  if (this != default_instance_) {
  }
}

const ::google::protobuf::Descriptor* Staff::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Staff_descriptor_;
}

const Staff& Staff::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_Testmsgs_2eproto();  return *default_instance_;
}

Staff* Staff::default_instance_ = NULL;

Staff* Staff::New() const {
  return new Staff;
}

void Staff::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (_has_bit(0)) {
      if (name_ != &_default_name_) {
        name_->clear();
      }
    }
    id_ = 0;
    if (_has_bit(2)) {
      if (email_ != &_default_email_) {
        email_->clear();
      }
    }
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Staff::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string name = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) !=
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
          goto handle_uninterpreted;
        }
        DO_(::google::protobuf::internal::WireFormatLite::ReadString(
              input, this->mutable_name()));
        ::google::protobuf::internal::WireFormat::VerifyUTF8String(
          this->name().data(), this->name().length(),
          ::google::protobuf::internal::WireFormat::PARSE);
        if (input->ExpectTag(16)) goto parse_id;
        break;
      }
      
      // required int32 id = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) !=
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
          goto handle_uninterpreted;
        }
       parse_id:
        DO_(::google::protobuf::internal::WireFormatLite::ReadInt32(
              input, &id_));
        _set_bit(1);
        if (input->ExpectTag(26)) goto parse_email;
        break;
      }
      
      // optional string email = 3;
      case 3: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) !=
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
          goto handle_uninterpreted;
        }
       parse_email:
        DO_(::google::protobuf::internal::WireFormatLite::ReadString(
              input, this->mutable_email()));
        ::google::protobuf::internal::WireFormat::VerifyUTF8String(
          this->email().data(), this->email().length(),
          ::google::protobuf::internal::WireFormat::PARSE);
        if (input->ExpectAtEnd()) return true;
        break;
      }
      
      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void Staff::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  ::google::protobuf::uint8* raw_buffer = output->GetDirectBufferForNBytesAndAdvance(_cached_size_);
  if (raw_buffer != NULL) {
    Staff::SerializeWithCachedSizesToArray(raw_buffer);
    return;
  }
  
  // required string name = 1;
  if (_has_bit(0)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    ::google::protobuf::internal::WireFormatLite::WriteString(
      1, this->name(), output);
  }
  
  // required int32 id = 2;
  if (_has_bit(1)) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(2, this->id(), output);
  }
  
  // optional string email = 3;
  if (_has_bit(2)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->email().data(), this->email().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    ::google::protobuf::internal::WireFormatLite::WriteString(
      3, this->email(), output);
  }
  
  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* Staff::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // required string name = 1;
  if (_has_bit(0)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->name(), target);
  }
  
  // required int32 id = 2;
  if (_has_bit(1)) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(2, this->id(), target);
  }
  
  // optional string email = 3;
  if (_has_bit(2)) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->email().data(), this->email().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        3, this->email(), target);
  }
  
  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int Staff::ByteSize() const {
  int total_size = 0;
  
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required string name = 1;
    if (has_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->name());
    }
    
    // required int32 id = 2;
    if (has_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->id());
    }
    
    // optional string email = 3;
    if (has_email()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->email());
    }
    
  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  _cached_size_ = total_size;
  return total_size;
}

void Staff::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Staff* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Staff*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Staff::MergeFrom(const Staff& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from._has_bit(0)) {
      set_name(from.name());
    }
    if (from._has_bit(1)) {
      set_id(from.id());
    }
    if (from._has_bit(2)) {
      set_email(from.email());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Staff::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Staff::CopyFrom(const Staff& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Staff::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;
  
  return true;
}

void Staff::Swap(Staff* other) {
  if (other != this) {
    std::swap(name_, other->name_);
    std::swap(id_, other->id_);
    std::swap(email_, other->email_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Staff::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Staff_descriptor_;
  metadata.reflection = Staff_reflection_;
  return metadata;
}
