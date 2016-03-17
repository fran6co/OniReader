#include <gtest/gtest.h>
#include "OniReader.h"

class Test : public ::testing::TestWithParam<const char*> {
};

TEST_P(Test, read)
{
    std::string path = getenv("ONI_TEST_DATA_PATH");

    OniReader reader (path + "/" + GetParam());

    ASSERT_TRUE(reader.isOpen());

    OniReader::Frame::Ptr frame;
    while(reader.read(frame)) {
    }
}

INSTANTIATE_TEST_CASE_P(ONIs, Test, ::testing::Values("1005.oni"));