//
// Created by Francisco Facioni on 14/3/16.
//

#include "OniReader.h"

#include <stdexcept>
#include <fstream>
#include <map>
#include <vector>

#include <jpeglib.h>

template <typename T>
T read(std::istream& stream){
    T value;
    stream.read(reinterpret_cast<char*>(&value), sizeof(T));

    return value;
}

std::string read(std::istream& stream, size_t size) {
    auto buffer = std::get_temporary_buffer<char>(size);
    stream.read(buffer.first, buffer.second);

    std::string ret (buffer.first, buffer.second);

    std::return_temporary_buffer(buffer.first);

    return ret;
}

template<>
std::string read<std::string>(std::istream& stream) {
    uint32_t size = read<uint32_t>(stream);

    return read(stream, size);
}

template<>
std::vector<uint8_t> read<std::vector<uint8_t>>(std::istream& stream) {
    uint32_t size = read<uint32_t>(stream);
    std::vector<uint8_t> blob (size);
    stream.read(reinterpret_cast<char*>(&blob[0]), size);

    return blob;
}

void readJpeg(unsigned char* buffer, size_t size, uint8_t* bitmap)
{
    struct jpeg_decompress_struct info; //for our jpeg info
    struct jpeg_error_mgr err; //the error handler

    info.err = jpeg_std_error( &err );
    jpeg_create_decompress( &info ); //fills info structure

    jpeg_mem_src(&info, buffer, size);
    jpeg_read_header( &info, true );

    jpeg_start_decompress( &info );

    int w = info.output_width;
    int h = info.output_height;
    int numChannels = info.num_components; // 3 = RGB, 4 = RGBA

    while ( info.output_scanline < h )
    {
        uint8_t* rowptr = bitmap + info.output_scanline * w * numChannels;
        jpeg_read_scanlines( &info, &rowptr, 1 );
    }

    jpeg_finish_decompress( &info );
    jpeg_destroy_decompress( &info );
}

struct membuf : std::streambuf
{
    membuf(char* begin, size_t size) {
        this->setg(begin, begin, begin+size);
    }
};

void decodeDepthWithEmbeddedTable(char *depthStream, size_t size, uint16_t* output,  size_t outputSize) {
    membuf buffer (depthStream, size);
    std::istream input(&buffer);

    uint16_t depthTableSize = read<uint16_t>(input);
    std::vector<uint16_t> depthTable;
    depthTable.reserve(depthTableSize);
    for(uint16_t i=0;i<depthTableSize;i++){
        depthTable.push_back(read<uint16_t>(input));
    }

    const uint16_t* outputEnd = &output[outputSize];

    uint16_t lastDepth = read<uint16_t>(input);
    while(!input.eof()) {
        uint8_t byte = read<uint8_t>(input);
        if (byte < 0xE0) {
            lastDepth -= (byte >> 4) - 6;
            *output++ = depthTable[lastDepth];

            if (output == outputEnd) {
                break;
            }

            uint8_t inData = (byte & 0x0F);
            if (inData != 0x0F) {
                if (inData != 0x0D) {
                    lastDepth -= (inData - 6);
                    *output++ =  depthTable[lastDepth];
                }
            } else {
                inData = read<uint8_t>(input);
                if (inData & 0x80) {
                    lastDepth -= (inData - 192);
                    *output++ = depthTable[lastDepth];
                } else {
                    lastDepth = inData << 8;
                    lastDepth += read<uint8_t>(input);
                    *output++ = depthTable[lastDepth];
                }
            }
        } else if (byte == 0xFF) {
            uint8_t inData = read<uint8_t>(input);
            if (inData & 0x80) {
                lastDepth -= (inData - 192);
                *output++ = depthTable[lastDepth];
            } else {
                lastDepth = inData << 8;
                lastDepth += read<uint8_t>(input);
                *output++ = depthTable[lastDepth];
            }
        } else {//It must be 0xE?
            uint16_t depth = depthTable[lastDepth];
            uint8_t zeros = byte-0xE0;
            for(int i=0;i<zeros;i++){
                *output++ = depth;
                // ONIs have sometimes an extra 0 at the end
                if (output == outputEnd) {
                    break;
                }
                *output++ = depth;
            }
        }
    }
}

#pragma pack(push, 1)
struct Version {
    uint8_t major;
    uint8_t minor;
    uint16_t maintenance;
    uint32_t build;
};

struct RecordingHeader
{
    uint32_t magic;
    Version version;
    uint64_t globalMaxTimeStamp;
    uint32_t maxNodeId;
};
#pragma pack(pop)

enum RecordType
{
    RECORD_NODE_ADDED_1_0_0_4		= 0x02,
    RECORD_INT_PROPERTY				= 0x03,
    RECORD_REAL_PROPERTY			= 0x04,
    RECORD_STRING_PROPERTY			= 0x05,
    RECORD_GENERAL_PROPERTY			= 0x06,
    RECORD_NODE_REMOVED				= 0x07,
    RECORD_NODE_DATA_BEGIN			= 0x08,
    RECORD_NODE_STATE_READY			= 0x09,
    RECORD_NEW_DATA					= 0x0A,
    RECORD_END						= 0x0B,
    RECORD_NODE_ADDED_1_0_0_5		= 0x0C,
    RECORD_NODE_ADDED				= 0x0D,
    RECORD_SEEK_TABLE               = 0x0E,
};

class Record {
public:
    typedef std::shared_ptr<Record> Ptr;

#pragma pack(push, 1)
    struct BaseHeader
    {
        uint32_t magic;
        uint32_t recordType;
        uint32_t nodeId;
        uint32_t fieldsSize;
        uint32_t payloadSize;
    };
    struct Header32 : public BaseHeader
    {
        uint32_t undoRecordPos;
    };
    struct Header64 : public BaseHeader
    {
        uint64_t undoRecordPos;
    };
#pragma pack(pop)

    virtual ~Record() = default;
};

class RecordNodeAdded : public Record {
public:
    typedef std::shared_ptr<RecordNodeAdded> Ptr;
#pragma pack(push, 1)
    struct Base32 {
        uint32_t type;
        char codecId[4];
        uint32_t numberOfFrames;
        uint64_t minTimestamp;
        uint64_t maxTimestamp;
    };

    struct Base64 : public Base32 {
        uint64_t seekTablePosition;
    };
#pragma pack(pop)

    RecordNodeAdded(uint32_t _id, const std::string& _codec)
            : id(_id), codec(_codec){

    }

    uint32_t getId() const {
        return id;
    }

    const std::string& getCodec() const {
        return codec;
    }

    uint32_t id;
    std::string codec;
};

class RecordSeekTable : public Record {
public:
};

class RecordNodeDataBegin : public Record {
public:

#pragma pack(push, 1)
    struct Base
    {
        uint32_t numberOfFrames;
        uint64_t maxTimeStamp;
    };
#pragma pack(pop)

};

class BaseProperty : public Record {
public:
    typedef std::shared_ptr<BaseProperty> Ptr;

    virtual ~BaseProperty() = default;

    virtual const std::string& getName() const = 0;
};

template<typename T>
class Property : public BaseProperty {
public:
    Property(const std::string& _name, const T& _value) :
            name(_name), value(_value) {
    }

    const std::string& getName() const {
        return name;
    }

    std::string name;
    T value;
};

class RecordFrame : public Record {
public:
    typedef std::shared_ptr<RecordFrame> Ptr;

#pragma pack(push, 1)
    struct Base {
        uint64_t timestamp;
        uint32_t frameNumber;
    };
#pragma pack(pop)


    RecordFrame(uint32_t _nodeId, uint64_t _timestamp, const std::vector<char>& _compressed)
            : nodeId(_nodeId), timestamp(_timestamp), compressed(_compressed) {
    }

    uint32_t getNodeId() const {
        return nodeId;
    }

    uint32_t nodeId;
    uint64_t timestamp;
    std::vector<char> compressed;
};

class RecordEnd : public Record {
public:
};

Record::Ptr readRecord(std::istream& stream, bool old32Headers) {
    Record::BaseHeader header;
    if (old32Headers) {
        header = read<Record::Header32>(stream);
    } else {
        header = read<Record::Header64>(stream);
    }

    // NIR
    if (header.magic != 0x0052494E) {
        throw std::runtime_error("Invalid record");
    }

    Record::Ptr record = std::make_shared<Record>();

    switch(header.recordType) {
        case RECORD_NODE_ADDED_1_0_0_5: {
            std::string nodeName = read<std::string>(stream);
            RecordNodeAdded::Base32 nodeAdded = read<RecordNodeAdded::Base32>(stream);
            record = std::make_shared<RecordNodeAdded>(header.nodeId, std::string(nodeAdded.codecId, sizeof(nodeAdded.codecId)));
            break;
        }
        case RECORD_NODE_ADDED: {
            std::string nodeName = read<std::string>(stream);
            RecordNodeAdded::Base64 nodeAdded = read<RecordNodeAdded::Base64>(stream);
            if (nodeAdded.numberOfFrames > 0 && nodeAdded.seekTablePosition != 0) {
                auto currentPosition = stream.tellg();
                stream.seekg(nodeAdded.seekTablePosition, std::ios_base::beg);
                Record::Ptr dataIndex = read<Record::Ptr>(stream);
                stream.seekg(currentPosition, std::ios_base::beg);
            }
            record = std::make_shared<RecordNodeAdded>(header.nodeId, std::string(nodeAdded.codecId, sizeof(nodeAdded.codecId)));
            break;
        }
        case RECORD_SEEK_TABLE: {
            record = std::make_shared<RecordSeekTable>();
            break;
        }
        case RECORD_NODE_DATA_BEGIN: {
            RecordNodeDataBegin::Base nodeDataBegin = read<RecordNodeDataBegin::Base>(stream);
            record = std::make_shared<RecordNodeDataBegin>();
            break;
        }
        case RECORD_GENERAL_PROPERTY: {
            std::string propertyName = std::string(read<std::string>(stream).c_str());
            std::vector<uint8_t> blob = read<std::vector<uint8_t>>(stream);
            record = std::make_shared<Property<std::vector<uint8_t>>>(propertyName, blob);
            break;
        }
        case RECORD_STRING_PROPERTY: {
            std::string propertyName = std::string(read<std::string>(stream).c_str());
            std::string value = read<std::string>(stream);
            record = std::make_shared<Property<std::string>>(propertyName, value);
            break;
        }
        case RECORD_REAL_PROPERTY: {
            std::string propertyName = std::string(read<std::string>(stream).c_str());
            uint32_t propertySize = read<uint32_t>(stream);
            double value = read<double>(stream);
            record = std::make_shared<Property<double>>(propertyName, value);
            break;
        }
        case RECORD_INT_PROPERTY: {
            std::string propertyName = std::string(read<std::string>(stream).c_str());
            uint32_t propertySize = read<uint32_t>(stream);
            uint64_t value = read<uint64_t>(stream);
            record = std::make_shared<Property<uint64_t>>(propertyName, value);
            break;
        }
        case RECORD_NODE_STATE_READY: {
            break;
        }
        case RECORD_NEW_DATA: {
            RecordFrame::Base frame = read<RecordFrame::Base>(stream);
            std::vector<char> compressed (header.payloadSize);
            stream.read(reinterpret_cast<char*>(&compressed[0]), header.payloadSize);
            record = std::make_shared<RecordFrame>(header.nodeId, frame.timestamp, compressed);
            break;
        }
        case RECORD_END: {
            record = std::make_shared<RecordEnd>();
            break;
        }
        default: {
            break;
        }
    }

    return record;
}

class OniReader::Impl {
public:
    Impl(const std::string &file)
        : stream (file, std::ios_base::in | std::ios_base::binary)
        , width (640)
        , height (480)
        , open(false)
        , old32Headers(true)
    {
        open = stream.is_open();

        if (!open) {
            return;
        }

        RecordingHeader header = ::read<RecordingHeader>(stream);

        if (header.magic != 0x3031494E) {
            open = false;
            return;
        }

        // We support versions ~1.0
        if (header.version.major != 1 || header.version.minor != 0) {
            open = false;
            return;
        }

        if (header.version.major == 1 && header.version.minor == 0 && header.version.minor >= 1) {
            old32Headers = false;
        }

        while (true) {
            Record::Ptr record;
            try {
                record = readRecord(stream, old32Headers);
            } catch (std::exception& e) {
                open = false;
                break;
            }

            if (std::dynamic_pointer_cast<RecordNodeDataBegin>(record)) {
                break;
            }

            if (std::dynamic_pointer_cast<BaseProperty>(record)) {
                BaseProperty::Ptr property = std::dynamic_pointer_cast<BaseProperty>(record);
                if (property->getName() == "ZPPS") {
                    zeroPlanePixelSize = std::dynamic_pointer_cast<Property<double>>(record)->value;
                } else if (property->getName() == "ZPD") {
                    zeroPlaneDistance = std::dynamic_pointer_cast<Property<uint64_t>>(record)->value;
                } else if (property->getName() == "Resolution") {
                    uint64_t resolution = std::dynamic_pointer_cast<Property<uint64_t>>(record)->value;
                    if (resolution != 1) {
                        open = false;
                        return;
                    }
                }
            } else if (std::dynamic_pointer_cast<RecordNodeAdded>(record)) {
                RecordNodeAdded::Ptr node = std::dynamic_pointer_cast<RecordNodeAdded>(record);
                nodes[node->getId()] = node->getCodec();
            }
        }
    }

    bool read(std::shared_ptr<Frame>& frame) {
        if (!frame || frame->width != width || frame->height != height || frame->zeroPlanePixelSize != zeroPlanePixelSize || frame->zeroPlaneDistance != zeroPlaneDistance) {
            frame = std::make_shared<Frame>(width, height, zeroPlanePixelSize, zeroPlaneDistance);
        }

        bool newRgb = false, newDepth = false;
        while(true) {
            Record::Ptr record = readRecord(stream, old32Headers);

            if (std::dynamic_pointer_cast<RecordEnd>(record)) {
                return false;
            } else if (std::dynamic_pointer_cast<RecordFrame>(record)) {
                RecordFrame::Ptr oniFrame = std::dynamic_pointer_cast<RecordFrame>(record);
                std::string codecId = nodes[oniFrame->getNodeId()];
                frame->timestamp = std::max(std::chrono::microseconds(oniFrame->timestamp), frame->timestamp);

                if (codecId == "JPEG") {
                    readJpeg(reinterpret_cast<unsigned char*>(&oniFrame->compressed[0]), oniFrame->compressed.size(), frame->rgb.get());
                    newRgb = true;
                } else if (codecId == "16zT") {
                    decodeDepthWithEmbeddedTable(&oniFrame->compressed[0], oniFrame->compressed.size(), frame->depth.get(), frame->width*frame->height);
                    newDepth = true;
                }
            }

            if (newRgb && newDepth) {
                break;
            }
        }

        return true;
    }

    std::ifstream stream;
    std::map<uint32_t, std::string> nodes;
    double zeroPlanePixelSize;
    uint64_t zeroPlaneDistance;
    uint32_t width, height;
    bool old32Headers;
    bool open;
};


OniReader::OniReader(const std::string &file)
    : impl(new Impl(file))
{
}

bool OniReader::isOpen() const {
    return impl->open;
}

bool OniReader::read(std::shared_ptr<OniReader::Frame>& frame) {
    return impl->read(frame);
}