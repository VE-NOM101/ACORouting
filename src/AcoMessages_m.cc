//
// Generated file, do not edit! Created by opp_msgtool 6.2 from AcoMessages.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wshadow"
#  pragma clang diagnostic ignored "-Wconversion"
#  pragma clang diagnostic ignored "-Wunused-parameter"
#  pragma clang diagnostic ignored "-Wc++98-compat"
#  pragma clang diagnostic ignored "-Wunreachable-code-break"
#  pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wshadow"
#  pragma GCC diagnostic ignored "-Wconversion"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wold-style-cast"
#  pragma GCC diagnostic ignored "-Wsuggest-attribute=noreturn"
#  pragma GCC diagnostic ignored "-Wfloat-conversion"
#endif

#include <iostream>
#include <sstream>
#include <memory>
#include <type_traits>
#include "AcoMessages_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp

Register_Class(DataMsg)

DataMsg::DataMsg(const char *name, short kind) : ::omnetpp::cPacket(name, kind)
{
}

DataMsg::DataMsg(const DataMsg& other) : ::omnetpp::cPacket(other)
{
    copy(other);
}

DataMsg::~DataMsg()
{
}

DataMsg& DataMsg::operator=(const DataMsg& other)
{
    if (this == &other) return *this;
    ::omnetpp::cPacket::operator=(other);
    copy(other);
    return *this;
}

void DataMsg::copy(const DataMsg& other)
{
    this->srcAddress = other.srcAddress;
    this->destAddress = other.destAddress;
    this->hopCount = other.hopCount;
    this->payload = other.payload;
}

void DataMsg::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::omnetpp::cPacket::parsimPack(b);
    doParsimPacking(b,this->srcAddress);
    doParsimPacking(b,this->destAddress);
    doParsimPacking(b,this->hopCount);
    doParsimPacking(b,this->payload);
}

void DataMsg::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::omnetpp::cPacket::parsimUnpack(b);
    doParsimUnpacking(b,this->srcAddress);
    doParsimUnpacking(b,this->destAddress);
    doParsimUnpacking(b,this->hopCount);
    doParsimUnpacking(b,this->payload);
}

int DataMsg::getSrcAddress() const
{
    return this->srcAddress;
}

void DataMsg::setSrcAddress(int srcAddress)
{
    this->srcAddress = srcAddress;
}

int DataMsg::getDestAddress() const
{
    return this->destAddress;
}

void DataMsg::setDestAddress(int destAddress)
{
    this->destAddress = destAddress;
}

int DataMsg::getHopCount() const
{
    return this->hopCount;
}

void DataMsg::setHopCount(int hopCount)
{
    this->hopCount = hopCount;
}

const char * DataMsg::getPayload() const
{
    return this->payload.c_str();
}

void DataMsg::setPayload(const char * payload)
{
    this->payload = payload;
}

class DataMsgDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertyNames;
    enum FieldConstants {
        FIELD_srcAddress,
        FIELD_destAddress,
        FIELD_hopCount,
        FIELD_payload,
    };
  public:
    DataMsgDescriptor();
    virtual ~DataMsgDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyName) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyName) const override;
    virtual int getFieldArraySize(omnetpp::any_ptr object, int field) const override;
    virtual void setFieldArraySize(omnetpp::any_ptr object, int field, int size) const override;

    virtual const char *getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const override;
    virtual std::string getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const override;
    virtual omnetpp::cValue getFieldValue(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual omnetpp::any_ptr getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const override;
};

Register_ClassDescriptor(DataMsgDescriptor)

DataMsgDescriptor::DataMsgDescriptor() : omnetpp::cClassDescriptor(omnetpp::opp_typename(typeid(DataMsg)), "omnetpp::cPacket")
{
    propertyNames = nullptr;
}

DataMsgDescriptor::~DataMsgDescriptor()
{
    delete[] propertyNames;
}

bool DataMsgDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<DataMsg *>(obj)!=nullptr;
}

const char **DataMsgDescriptor::getPropertyNames() const
{
    if (!propertyNames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
        const char **baseNames = base ? base->getPropertyNames() : nullptr;
        propertyNames = mergeLists(baseNames, names);
    }
    return propertyNames;
}

const char *DataMsgDescriptor::getProperty(const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? base->getProperty(propertyName) : nullptr;
}

int DataMsgDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? 4+base->getFieldCount() : 4;
}

unsigned int DataMsgDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeFlags(field);
        field -= base->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,    // FIELD_srcAddress
        FD_ISEDITABLE,    // FIELD_destAddress
        FD_ISEDITABLE,    // FIELD_hopCount
        FD_ISEDITABLE,    // FIELD_payload
    };
    return (field >= 0 && field < 4) ? fieldTypeFlags[field] : 0;
}

const char *DataMsgDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldName(field);
        field -= base->getFieldCount();
    }
    static const char *fieldNames[] = {
        "srcAddress",
        "destAddress",
        "hopCount",
        "payload",
    };
    return (field >= 0 && field < 4) ? fieldNames[field] : nullptr;
}

int DataMsgDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    int baseIndex = base ? base->getFieldCount() : 0;
    if (strcmp(fieldName, "srcAddress") == 0) return baseIndex + 0;
    if (strcmp(fieldName, "destAddress") == 0) return baseIndex + 1;
    if (strcmp(fieldName, "hopCount") == 0) return baseIndex + 2;
    if (strcmp(fieldName, "payload") == 0) return baseIndex + 3;
    return base ? base->findField(fieldName) : -1;
}

const char *DataMsgDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeString(field);
        field -= base->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "int",    // FIELD_srcAddress
        "int",    // FIELD_destAddress
        "int",    // FIELD_hopCount
        "string",    // FIELD_payload
    };
    return (field >= 0 && field < 4) ? fieldTypeStrings[field] : nullptr;
}

const char **DataMsgDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldPropertyNames(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *DataMsgDescriptor::getFieldProperty(int field, const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldProperty(field, propertyName);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int DataMsgDescriptor::getFieldArraySize(omnetpp::any_ptr object, int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldArraySize(object, field);
        field -= base->getFieldCount();
    }
    DataMsg *pp = omnetpp::fromAnyPtr<DataMsg>(object); (void)pp;
    switch (field) {
        default: return 0;
    }
}

void DataMsgDescriptor::setFieldArraySize(omnetpp::any_ptr object, int field, int size) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldArraySize(object, field, size);
            return;
        }
        field -= base->getFieldCount();
    }
    DataMsg *pp = omnetpp::fromAnyPtr<DataMsg>(object); (void)pp;
    switch (field) {
        default: throw omnetpp::cRuntimeError("Cannot set array size of field %d of class 'DataMsg'", field);
    }
}

const char *DataMsgDescriptor::getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldDynamicTypeString(object,field,i);
        field -= base->getFieldCount();
    }
    DataMsg *pp = omnetpp::fromAnyPtr<DataMsg>(object); (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

std::string DataMsgDescriptor::getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValueAsString(object,field,i);
        field -= base->getFieldCount();
    }
    DataMsg *pp = omnetpp::fromAnyPtr<DataMsg>(object); (void)pp;
    switch (field) {
        case FIELD_srcAddress: return long2string(pp->getSrcAddress());
        case FIELD_destAddress: return long2string(pp->getDestAddress());
        case FIELD_hopCount: return long2string(pp->getHopCount());
        case FIELD_payload: return oppstring2string(pp->getPayload());
        default: return "";
    }
}

void DataMsgDescriptor::setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValueAsString(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    DataMsg *pp = omnetpp::fromAnyPtr<DataMsg>(object); (void)pp;
    switch (field) {
        case FIELD_srcAddress: pp->setSrcAddress(string2long(value)); break;
        case FIELD_destAddress: pp->setDestAddress(string2long(value)); break;
        case FIELD_hopCount: pp->setHopCount(string2long(value)); break;
        case FIELD_payload: pp->setPayload((value)); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'DataMsg'", field);
    }
}

omnetpp::cValue DataMsgDescriptor::getFieldValue(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValue(object,field,i);
        field -= base->getFieldCount();
    }
    DataMsg *pp = omnetpp::fromAnyPtr<DataMsg>(object); (void)pp;
    switch (field) {
        case FIELD_srcAddress: return pp->getSrcAddress();
        case FIELD_destAddress: return pp->getDestAddress();
        case FIELD_hopCount: return pp->getHopCount();
        case FIELD_payload: return pp->getPayload();
        default: throw omnetpp::cRuntimeError("Cannot return field %d of class 'DataMsg' as cValue -- field index out of range?", field);
    }
}

void DataMsgDescriptor::setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValue(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    DataMsg *pp = omnetpp::fromAnyPtr<DataMsg>(object); (void)pp;
    switch (field) {
        case FIELD_srcAddress: pp->setSrcAddress(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_destAddress: pp->setDestAddress(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_hopCount: pp->setHopCount(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_payload: pp->setPayload(value.stringValue()); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'DataMsg'", field);
    }
}

const char *DataMsgDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructName(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    };
}

omnetpp::any_ptr DataMsgDescriptor::getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructValuePointer(object, field, i);
        field -= base->getFieldCount();
    }
    DataMsg *pp = omnetpp::fromAnyPtr<DataMsg>(object); (void)pp;
    switch (field) {
        default: return omnetpp::any_ptr(nullptr);
    }
}

void DataMsgDescriptor::setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldStructValuePointer(object, field, i, ptr);
            return;
        }
        field -= base->getFieldCount();
    }
    DataMsg *pp = omnetpp::fromAnyPtr<DataMsg>(object); (void)pp;
    switch (field) {
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'DataMsg'", field);
    }
}

Register_Class(AntMsg)

AntMsg::AntMsg(const char *name, short kind) : ::omnetpp::cPacket(name, kind)
{
}

AntMsg::AntMsg(const AntMsg& other) : ::omnetpp::cPacket(other)
{
    copy(other);
}

AntMsg::~AntMsg()
{
    delete [] this->visitedNodes;
}

AntMsg& AntMsg::operator=(const AntMsg& other)
{
    if (this == &other) return *this;
    ::omnetpp::cPacket::operator=(other);
    copy(other);
    return *this;
}

void AntMsg::copy(const AntMsg& other)
{
    this->srcAddress = other.srcAddress;
    this->destAddress = other.destAddress;
    this->hopCount = other.hopCount;
    this->isForwardAnt_ = other.isForwardAnt_;
    delete [] this->visitedNodes;
    this->visitedNodes = (other.visitedNodes_arraysize==0) ? nullptr : new int[other.visitedNodes_arraysize];
    visitedNodes_arraysize = other.visitedNodes_arraysize;
    for (size_t i = 0; i < visitedNodes_arraysize; i++) {
        this->visitedNodes[i] = other.visitedNodes[i];
    }
    this->pathCost = other.pathCost;
}

void AntMsg::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::omnetpp::cPacket::parsimPack(b);
    doParsimPacking(b,this->srcAddress);
    doParsimPacking(b,this->destAddress);
    doParsimPacking(b,this->hopCount);
    doParsimPacking(b,this->isForwardAnt_);
    b->pack(visitedNodes_arraysize);
    doParsimArrayPacking(b,this->visitedNodes,visitedNodes_arraysize);
    doParsimPacking(b,this->pathCost);
}

void AntMsg::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::omnetpp::cPacket::parsimUnpack(b);
    doParsimUnpacking(b,this->srcAddress);
    doParsimUnpacking(b,this->destAddress);
    doParsimUnpacking(b,this->hopCount);
    doParsimUnpacking(b,this->isForwardAnt_);
    delete [] this->visitedNodes;
    b->unpack(visitedNodes_arraysize);
    if (visitedNodes_arraysize == 0) {
        this->visitedNodes = nullptr;
    } else {
        this->visitedNodes = new int[visitedNodes_arraysize];
        doParsimArrayUnpacking(b,this->visitedNodes,visitedNodes_arraysize);
    }
    doParsimUnpacking(b,this->pathCost);
}

int AntMsg::getSrcAddress() const
{
    return this->srcAddress;
}

void AntMsg::setSrcAddress(int srcAddress)
{
    this->srcAddress = srcAddress;
}

int AntMsg::getDestAddress() const
{
    return this->destAddress;
}

void AntMsg::setDestAddress(int destAddress)
{
    this->destAddress = destAddress;
}

int AntMsg::getHopCount() const
{
    return this->hopCount;
}

void AntMsg::setHopCount(int hopCount)
{
    this->hopCount = hopCount;
}

bool AntMsg::isForwardAnt() const
{
    return this->isForwardAnt_;
}

void AntMsg::setIsForwardAnt(bool isForwardAnt)
{
    this->isForwardAnt_ = isForwardAnt;
}

size_t AntMsg::getVisitedNodesArraySize() const
{
    return visitedNodes_arraysize;
}

int AntMsg::getVisitedNodes(size_t k) const
{
    if (k >= visitedNodes_arraysize) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)visitedNodes_arraysize, (unsigned long)k);
    return this->visitedNodes[k];
}

void AntMsg::setVisitedNodesArraySize(size_t newSize)
{
    int *visitedNodes2 = (newSize==0) ? nullptr : new int[newSize];
    size_t minSize = visitedNodes_arraysize < newSize ? visitedNodes_arraysize : newSize;
    for (size_t i = 0; i < minSize; i++)
        visitedNodes2[i] = this->visitedNodes[i];
    for (size_t i = minSize; i < newSize; i++)
        visitedNodes2[i] = 0;
    delete [] this->visitedNodes;
    this->visitedNodes = visitedNodes2;
    visitedNodes_arraysize = newSize;
}

void AntMsg::setVisitedNodes(size_t k, int visitedNodes)
{
    if (k >= visitedNodes_arraysize) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)visitedNodes_arraysize, (unsigned long)k);
    this->visitedNodes[k] = visitedNodes;
}

void AntMsg::insertVisitedNodes(size_t k, int visitedNodes)
{
    if (k > visitedNodes_arraysize) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)visitedNodes_arraysize, (unsigned long)k);
    size_t newSize = visitedNodes_arraysize + 1;
    int *visitedNodes2 = new int[newSize];
    size_t i;
    for (i = 0; i < k; i++)
        visitedNodes2[i] = this->visitedNodes[i];
    visitedNodes2[k] = visitedNodes;
    for (i = k + 1; i < newSize; i++)
        visitedNodes2[i] = this->visitedNodes[i-1];
    delete [] this->visitedNodes;
    this->visitedNodes = visitedNodes2;
    visitedNodes_arraysize = newSize;
}

void AntMsg::appendVisitedNodes(int visitedNodes)
{
    insertVisitedNodes(visitedNodes_arraysize, visitedNodes);
}

void AntMsg::eraseVisitedNodes(size_t k)
{
    if (k >= visitedNodes_arraysize) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)visitedNodes_arraysize, (unsigned long)k);
    size_t newSize = visitedNodes_arraysize - 1;
    int *visitedNodes2 = (newSize == 0) ? nullptr : new int[newSize];
    size_t i;
    for (i = 0; i < k; i++)
        visitedNodes2[i] = this->visitedNodes[i];
    for (i = k; i < newSize; i++)
        visitedNodes2[i] = this->visitedNodes[i+1];
    delete [] this->visitedNodes;
    this->visitedNodes = visitedNodes2;
    visitedNodes_arraysize = newSize;
}

double AntMsg::getPathCost() const
{
    return this->pathCost;
}

void AntMsg::setPathCost(double pathCost)
{
    this->pathCost = pathCost;
}

class AntMsgDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertyNames;
    enum FieldConstants {
        FIELD_srcAddress,
        FIELD_destAddress,
        FIELD_hopCount,
        FIELD_isForwardAnt,
        FIELD_visitedNodes,
        FIELD_pathCost,
    };
  public:
    AntMsgDescriptor();
    virtual ~AntMsgDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyName) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyName) const override;
    virtual int getFieldArraySize(omnetpp::any_ptr object, int field) const override;
    virtual void setFieldArraySize(omnetpp::any_ptr object, int field, int size) const override;

    virtual const char *getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const override;
    virtual std::string getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const override;
    virtual omnetpp::cValue getFieldValue(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual omnetpp::any_ptr getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const override;
};

Register_ClassDescriptor(AntMsgDescriptor)

AntMsgDescriptor::AntMsgDescriptor() : omnetpp::cClassDescriptor(omnetpp::opp_typename(typeid(AntMsg)), "omnetpp::cPacket")
{
    propertyNames = nullptr;
}

AntMsgDescriptor::~AntMsgDescriptor()
{
    delete[] propertyNames;
}

bool AntMsgDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<AntMsg *>(obj)!=nullptr;
}

const char **AntMsgDescriptor::getPropertyNames() const
{
    if (!propertyNames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
        const char **baseNames = base ? base->getPropertyNames() : nullptr;
        propertyNames = mergeLists(baseNames, names);
    }
    return propertyNames;
}

const char *AntMsgDescriptor::getProperty(const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? base->getProperty(propertyName) : nullptr;
}

int AntMsgDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? 6+base->getFieldCount() : 6;
}

unsigned int AntMsgDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeFlags(field);
        field -= base->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,    // FIELD_srcAddress
        FD_ISEDITABLE,    // FIELD_destAddress
        FD_ISEDITABLE,    // FIELD_hopCount
        FD_ISEDITABLE,    // FIELD_isForwardAnt
        FD_ISARRAY | FD_ISEDITABLE | FD_ISRESIZABLE,    // FIELD_visitedNodes
        FD_ISEDITABLE,    // FIELD_pathCost
    };
    return (field >= 0 && field < 6) ? fieldTypeFlags[field] : 0;
}

const char *AntMsgDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldName(field);
        field -= base->getFieldCount();
    }
    static const char *fieldNames[] = {
        "srcAddress",
        "destAddress",
        "hopCount",
        "isForwardAnt",
        "visitedNodes",
        "pathCost",
    };
    return (field >= 0 && field < 6) ? fieldNames[field] : nullptr;
}

int AntMsgDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    int baseIndex = base ? base->getFieldCount() : 0;
    if (strcmp(fieldName, "srcAddress") == 0) return baseIndex + 0;
    if (strcmp(fieldName, "destAddress") == 0) return baseIndex + 1;
    if (strcmp(fieldName, "hopCount") == 0) return baseIndex + 2;
    if (strcmp(fieldName, "isForwardAnt") == 0) return baseIndex + 3;
    if (strcmp(fieldName, "visitedNodes") == 0) return baseIndex + 4;
    if (strcmp(fieldName, "pathCost") == 0) return baseIndex + 5;
    return base ? base->findField(fieldName) : -1;
}

const char *AntMsgDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeString(field);
        field -= base->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "int",    // FIELD_srcAddress
        "int",    // FIELD_destAddress
        "int",    // FIELD_hopCount
        "bool",    // FIELD_isForwardAnt
        "int",    // FIELD_visitedNodes
        "double",    // FIELD_pathCost
    };
    return (field >= 0 && field < 6) ? fieldTypeStrings[field] : nullptr;
}

const char **AntMsgDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldPropertyNames(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *AntMsgDescriptor::getFieldProperty(int field, const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldProperty(field, propertyName);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int AntMsgDescriptor::getFieldArraySize(omnetpp::any_ptr object, int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldArraySize(object, field);
        field -= base->getFieldCount();
    }
    AntMsg *pp = omnetpp::fromAnyPtr<AntMsg>(object); (void)pp;
    switch (field) {
        case FIELD_visitedNodes: return pp->getVisitedNodesArraySize();
        default: return 0;
    }
}

void AntMsgDescriptor::setFieldArraySize(omnetpp::any_ptr object, int field, int size) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldArraySize(object, field, size);
            return;
        }
        field -= base->getFieldCount();
    }
    AntMsg *pp = omnetpp::fromAnyPtr<AntMsg>(object); (void)pp;
    switch (field) {
        case FIELD_visitedNodes: pp->setVisitedNodesArraySize(size); break;
        default: throw omnetpp::cRuntimeError("Cannot set array size of field %d of class 'AntMsg'", field);
    }
}

const char *AntMsgDescriptor::getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldDynamicTypeString(object,field,i);
        field -= base->getFieldCount();
    }
    AntMsg *pp = omnetpp::fromAnyPtr<AntMsg>(object); (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

std::string AntMsgDescriptor::getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValueAsString(object,field,i);
        field -= base->getFieldCount();
    }
    AntMsg *pp = omnetpp::fromAnyPtr<AntMsg>(object); (void)pp;
    switch (field) {
        case FIELD_srcAddress: return long2string(pp->getSrcAddress());
        case FIELD_destAddress: return long2string(pp->getDestAddress());
        case FIELD_hopCount: return long2string(pp->getHopCount());
        case FIELD_isForwardAnt: return bool2string(pp->isForwardAnt());
        case FIELD_visitedNodes: return long2string(pp->getVisitedNodes(i));
        case FIELD_pathCost: return double2string(pp->getPathCost());
        default: return "";
    }
}

void AntMsgDescriptor::setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValueAsString(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    AntMsg *pp = omnetpp::fromAnyPtr<AntMsg>(object); (void)pp;
    switch (field) {
        case FIELD_srcAddress: pp->setSrcAddress(string2long(value)); break;
        case FIELD_destAddress: pp->setDestAddress(string2long(value)); break;
        case FIELD_hopCount: pp->setHopCount(string2long(value)); break;
        case FIELD_isForwardAnt: pp->setIsForwardAnt(string2bool(value)); break;
        case FIELD_visitedNodes: pp->setVisitedNodes(i,string2long(value)); break;
        case FIELD_pathCost: pp->setPathCost(string2double(value)); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'AntMsg'", field);
    }
}

omnetpp::cValue AntMsgDescriptor::getFieldValue(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValue(object,field,i);
        field -= base->getFieldCount();
    }
    AntMsg *pp = omnetpp::fromAnyPtr<AntMsg>(object); (void)pp;
    switch (field) {
        case FIELD_srcAddress: return pp->getSrcAddress();
        case FIELD_destAddress: return pp->getDestAddress();
        case FIELD_hopCount: return pp->getHopCount();
        case FIELD_isForwardAnt: return pp->isForwardAnt();
        case FIELD_visitedNodes: return pp->getVisitedNodes(i);
        case FIELD_pathCost: return pp->getPathCost();
        default: throw omnetpp::cRuntimeError("Cannot return field %d of class 'AntMsg' as cValue -- field index out of range?", field);
    }
}

void AntMsgDescriptor::setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValue(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    AntMsg *pp = omnetpp::fromAnyPtr<AntMsg>(object); (void)pp;
    switch (field) {
        case FIELD_srcAddress: pp->setSrcAddress(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_destAddress: pp->setDestAddress(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_hopCount: pp->setHopCount(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_isForwardAnt: pp->setIsForwardAnt(value.boolValue()); break;
        case FIELD_visitedNodes: pp->setVisitedNodes(i,omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_pathCost: pp->setPathCost(value.doubleValue()); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'AntMsg'", field);
    }
}

const char *AntMsgDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructName(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    };
}

omnetpp::any_ptr AntMsgDescriptor::getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructValuePointer(object, field, i);
        field -= base->getFieldCount();
    }
    AntMsg *pp = omnetpp::fromAnyPtr<AntMsg>(object); (void)pp;
    switch (field) {
        default: return omnetpp::any_ptr(nullptr);
    }
}

void AntMsgDescriptor::setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldStructValuePointer(object, field, i, ptr);
            return;
        }
        field -= base->getFieldCount();
    }
    AntMsg *pp = omnetpp::fromAnyPtr<AntMsg>(object); (void)pp;
    switch (field) {
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'AntMsg'", field);
    }
}

namespace omnetpp {

}  // namespace omnetpp

