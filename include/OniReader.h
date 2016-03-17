//
// Created by Francisco Facioni on 14/3/16.
//

#ifndef BACKGROUND_ONIREADER_H
#define BACKGROUND_ONIREADER_H

#include <memory>
#include <string>
#include <chrono>

class OniReader {
    class Impl;
    std::shared_ptr<Impl> impl;
public:
    class Frame {
    public:
        typedef std::shared_ptr<Frame> Ptr;
        typedef std::shared_ptr<const Frame> ConstPtr;

        Frame(uint32_t width, uint32_t height, double zeroPlanePixelSize, uint64_t zeroPlaneDistance)
            : width(width)
            , height(height)
            , zeroPlanePixelSize(zeroPlanePixelSize)
            , zeroPlaneDistance(zeroPlaneDistance)
            , rgb (new uint8_t[3*width*height])
            , depth (new uint16_t[width*height])
        {}

        const uint32_t width;
        const uint32_t height;
        const double zeroPlanePixelSize;
        const uint64_t zeroPlaneDistance;

        std::chrono::microseconds timestamp;
        const std::unique_ptr<uint8_t[]> rgb;
        const std::unique_ptr<uint16_t[]> depth;
    };

    OniReader(const std::string& file);

    bool isOpen() const;

    bool read(Frame::Ptr& frame);
};


#endif //BACKGROUND_ONIREADER_H
