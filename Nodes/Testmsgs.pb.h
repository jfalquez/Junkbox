// Generated by the protocol buffer compiler.  DO NOT EDIT!

#ifndef PROTOBUF_Testmsgs_2eproto__INCLUDED
#define PROTOBUF_Testmsgs_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2002000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2002000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_message_reflection.h>

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_Testmsgs_2eproto();
void protobuf_AssignDesc_Testmsgs_2eproto();
void protobuf_ShutdownFile_Testmsgs_2eproto();

class Professor;
class Student;
class Staff;

// ===================================================================

class Professor : public ::google::protobuf::Message {
 public:
  Professor();
  virtual ~Professor();
  
  Professor(const Professor& from);
  
  inline Professor& operator=(const Professor& from) {
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
  static const Professor& default_instance();
  void Swap(Professor* other);
  
  // implements Message ----------------------------------------------
  
  Professor* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Professor& from);
  void MergeFrom(const Professor& from);
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
  void SetCachedSize(int size) const { _cached_size_ = size; }
  public:
  
  ::google::protobuf::Metadata GetMetadata() const;
  
  // nested types ----------------------------------------------------
  
  // accessors -------------------------------------------------------
  
  // required string name = 1;
  inline bool has_name() const;
  inline void clear_name();
  static const int kNameFieldNumber = 1;
  inline const ::std::string& name() const;
  inline void set_name(const ::std::string& value);
  inline void set_name(const char* value);
  inline void set_name(const char* value, size_t size);
  inline ::std::string* mutable_name();
  
  // required int32 id = 2;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 2;
  inline ::google::protobuf::int32 id() const;
  inline void set_id(::google::protobuf::int32 value);
  
  // optional string email = 3;
  inline bool has_email() const;
  inline void clear_email();
  static const int kEmailFieldNumber = 3;
  inline const ::std::string& email() const;
  inline void set_email(const ::std::string& value);
  inline void set_email(const char* value);
  inline void set_email(const char* value, size_t size);
  inline ::std::string* mutable_email();
  
 private:
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  mutable int _cached_size_;
  
  ::std::string* name_;
  static const ::std::string _default_name_;
  ::google::protobuf::int32 id_;
  ::std::string* email_;
  static const ::std::string _default_email_;
  friend void  protobuf_AddDesc_Testmsgs_2eproto();
  friend void protobuf_AssignDesc_Testmsgs_2eproto();
  friend void protobuf_ShutdownFile_Testmsgs_2eproto();
  
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];
  
  // WHY DOES & HAVE LOWER PRECEDENCE THAN != !?
  inline bool _has_bit(int index) const {
    return (_has_bits_[index / 32] & (1u << (index % 32))) != 0;
  }
  inline void _set_bit(int index) {
    _has_bits_[index / 32] |= (1u << (index % 32));
  }
  inline void _clear_bit(int index) {
    _has_bits_[index / 32] &= ~(1u << (index % 32));
  }
  
  void InitAsDefaultInstance();
  static Professor* default_instance_;
};
// -------------------------------------------------------------------

class Student : public ::google::protobuf::Message {
 public:
  Student();
  virtual ~Student();
  
  Student(const Student& from);
  
  inline Student& operator=(const Student& from) {
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
  static const Student& default_instance();
  void Swap(Student* other);
  
  // implements Message ----------------------------------------------
  
  Student* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Student& from);
  void MergeFrom(const Student& from);
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
  void SetCachedSize(int size) const { _cached_size_ = size; }
  public:
  
  ::google::protobuf::Metadata GetMetadata() const;
  
  // nested types ----------------------------------------------------
  
  // accessors -------------------------------------------------------
  
  // required string name = 1;
  inline bool has_name() const;
  inline void clear_name();
  static const int kNameFieldNumber = 1;
  inline const ::std::string& name() const;
  inline void set_name(const ::std::string& value);
  inline void set_name(const char* value);
  inline void set_name(const char* value, size_t size);
  inline ::std::string* mutable_name();
  
  // required int32 id = 2;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 2;
  inline ::google::protobuf::int32 id() const;
  inline void set_id(::google::protobuf::int32 value);
  
  // optional string email = 3;
  inline bool has_email() const;
  inline void clear_email();
  static const int kEmailFieldNumber = 3;
  inline const ::std::string& email() const;
  inline void set_email(const ::std::string& value);
  inline void set_email(const char* value);
  inline void set_email(const char* value, size_t size);
  inline ::std::string* mutable_email();
  
 private:
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  mutable int _cached_size_;
  
  ::std::string* name_;
  static const ::std::string _default_name_;
  ::google::protobuf::int32 id_;
  ::std::string* email_;
  static const ::std::string _default_email_;
  friend void  protobuf_AddDesc_Testmsgs_2eproto();
  friend void protobuf_AssignDesc_Testmsgs_2eproto();
  friend void protobuf_ShutdownFile_Testmsgs_2eproto();
  
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];
  
  // WHY DOES & HAVE LOWER PRECEDENCE THAN != !?
  inline bool _has_bit(int index) const {
    return (_has_bits_[index / 32] & (1u << (index % 32))) != 0;
  }
  inline void _set_bit(int index) {
    _has_bits_[index / 32] |= (1u << (index % 32));
  }
  inline void _clear_bit(int index) {
    _has_bits_[index / 32] &= ~(1u << (index % 32));
  }
  
  void InitAsDefaultInstance();
  static Student* default_instance_;
};
// -------------------------------------------------------------------

class Staff : public ::google::protobuf::Message {
 public:
  Staff();
  virtual ~Staff();
  
  Staff(const Staff& from);
  
  inline Staff& operator=(const Staff& from) {
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
  static const Staff& default_instance();
  void Swap(Staff* other);
  
  // implements Message ----------------------------------------------
  
  Staff* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Staff& from);
  void MergeFrom(const Staff& from);
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
  void SetCachedSize(int size) const { _cached_size_ = size; }
  public:
  
  ::google::protobuf::Metadata GetMetadata() const;
  
  // nested types ----------------------------------------------------
  
  // accessors -------------------------------------------------------
  
  // required string name = 1;
  inline bool has_name() const;
  inline void clear_name();
  static const int kNameFieldNumber = 1;
  inline const ::std::string& name() const;
  inline void set_name(const ::std::string& value);
  inline void set_name(const char* value);
  inline void set_name(const char* value, size_t size);
  inline ::std::string* mutable_name();
  
  // required int32 id = 2;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 2;
  inline ::google::protobuf::int32 id() const;
  inline void set_id(::google::protobuf::int32 value);
  
  // optional string email = 3;
  inline bool has_email() const;
  inline void clear_email();
  static const int kEmailFieldNumber = 3;
  inline const ::std::string& email() const;
  inline void set_email(const ::std::string& value);
  inline void set_email(const char* value);
  inline void set_email(const char* value, size_t size);
  inline ::std::string* mutable_email();
  
 private:
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  mutable int _cached_size_;
  
  ::std::string* name_;
  static const ::std::string _default_name_;
  ::google::protobuf::int32 id_;
  ::std::string* email_;
  static const ::std::string _default_email_;
  friend void  protobuf_AddDesc_Testmsgs_2eproto();
  friend void protobuf_AssignDesc_Testmsgs_2eproto();
  friend void protobuf_ShutdownFile_Testmsgs_2eproto();
  
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];
  
  // WHY DOES & HAVE LOWER PRECEDENCE THAN != !?
  inline bool _has_bit(int index) const {
    return (_has_bits_[index / 32] & (1u << (index % 32))) != 0;
  }
  inline void _set_bit(int index) {
    _has_bits_[index / 32] |= (1u << (index % 32));
  }
  inline void _clear_bit(int index) {
    _has_bits_[index / 32] &= ~(1u << (index % 32));
  }
  
  void InitAsDefaultInstance();
  static Staff* default_instance_;
};
// ===================================================================


// ===================================================================


// ===================================================================

// Professor

// required string name = 1;
inline bool Professor::has_name() const {
  return _has_bit(0);
}
inline void Professor::clear_name() {
  if (name_ != &_default_name_) {
    name_->clear();
  }
  _clear_bit(0);
}
inline const ::std::string& Professor::name() const {
  return *name_;
}
inline void Professor::set_name(const ::std::string& value) {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Professor::set_name(const char* value) {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Professor::set_name(const char* value, size_t size) {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Professor::mutable_name() {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  return name_;
}

// required int32 id = 2;
inline bool Professor::has_id() const {
  return _has_bit(1);
}
inline void Professor::clear_id() {
  id_ = 0;
  _clear_bit(1);
}
inline ::google::protobuf::int32 Professor::id() const {
  return id_;
}
inline void Professor::set_id(::google::protobuf::int32 value) {
  _set_bit(1);
  id_ = value;
}

// optional string email = 3;
inline bool Professor::has_email() const {
  return _has_bit(2);
}
inline void Professor::clear_email() {
  if (email_ != &_default_email_) {
    email_->clear();
  }
  _clear_bit(2);
}
inline const ::std::string& Professor::email() const {
  return *email_;
}
inline void Professor::set_email(const ::std::string& value) {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  email_->assign(value);
}
inline void Professor::set_email(const char* value) {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  email_->assign(value);
}
inline void Professor::set_email(const char* value, size_t size) {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  email_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Professor::mutable_email() {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  return email_;
}

// -------------------------------------------------------------------

// Student

// required string name = 1;
inline bool Student::has_name() const {
  return _has_bit(0);
}
inline void Student::clear_name() {
  if (name_ != &_default_name_) {
    name_->clear();
  }
  _clear_bit(0);
}
inline const ::std::string& Student::name() const {
  return *name_;
}
inline void Student::set_name(const ::std::string& value) {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Student::set_name(const char* value) {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Student::set_name(const char* value, size_t size) {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Student::mutable_name() {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  return name_;
}

// required int32 id = 2;
inline bool Student::has_id() const {
  return _has_bit(1);
}
inline void Student::clear_id() {
  id_ = 0;
  _clear_bit(1);
}
inline ::google::protobuf::int32 Student::id() const {
  return id_;
}
inline void Student::set_id(::google::protobuf::int32 value) {
  _set_bit(1);
  id_ = value;
}

// optional string email = 3;
inline bool Student::has_email() const {
  return _has_bit(2);
}
inline void Student::clear_email() {
  if (email_ != &_default_email_) {
    email_->clear();
  }
  _clear_bit(2);
}
inline const ::std::string& Student::email() const {
  return *email_;
}
inline void Student::set_email(const ::std::string& value) {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  email_->assign(value);
}
inline void Student::set_email(const char* value) {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  email_->assign(value);
}
inline void Student::set_email(const char* value, size_t size) {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  email_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Student::mutable_email() {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  return email_;
}

// -------------------------------------------------------------------

// Staff

// required string name = 1;
inline bool Staff::has_name() const {
  return _has_bit(0);
}
inline void Staff::clear_name() {
  if (name_ != &_default_name_) {
    name_->clear();
  }
  _clear_bit(0);
}
inline const ::std::string& Staff::name() const {
  return *name_;
}
inline void Staff::set_name(const ::std::string& value) {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Staff::set_name(const char* value) {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Staff::set_name(const char* value, size_t size) {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Staff::mutable_name() {
  _set_bit(0);
  if (name_ == &_default_name_) {
    name_ = new ::std::string;
  }
  return name_;
}

// required int32 id = 2;
inline bool Staff::has_id() const {
  return _has_bit(1);
}
inline void Staff::clear_id() {
  id_ = 0;
  _clear_bit(1);
}
inline ::google::protobuf::int32 Staff::id() const {
  return id_;
}
inline void Staff::set_id(::google::protobuf::int32 value) {
  _set_bit(1);
  id_ = value;
}

// optional string email = 3;
inline bool Staff::has_email() const {
  return _has_bit(2);
}
inline void Staff::clear_email() {
  if (email_ != &_default_email_) {
    email_->clear();
  }
  _clear_bit(2);
}
inline const ::std::string& Staff::email() const {
  return *email_;
}
inline void Staff::set_email(const ::std::string& value) {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  email_->assign(value);
}
inline void Staff::set_email(const char* value) {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  email_->assign(value);
}
inline void Staff::set_email(const char* value, size_t size) {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  email_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Staff::mutable_email() {
  _set_bit(2);
  if (email_ == &_default_email_) {
    email_ = new ::std::string;
  }
  return email_;
}


#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

#endif  // PROTOBUF_Testmsgs_2eproto__INCLUDED
