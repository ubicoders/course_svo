#ifndef IOX2_EXAMPLES_MESSAGE_DATA_HPP
#define IOX2_EXAMPLES_MESSAGE_DATA_HPP

#include <cstdint>
#include <iostream>
#include <array>
#include <algorithm>

struct GenericHeader {
    // IOX2_TYPE_NAME is equivalent to the user header type name used on the Rust side
    static constexpr const char* IOX2_TYPE_NAME = "GenericHeader";
    int32_t frame_id;
    uint64_t timestamp;
};

inline auto operator<<(std::ostream& stream, const GenericHeader& value) -> std::ostream& {
    stream << "GenericHeader { frame_id: " << value.frame_id << ", timestamp: " << value.timestamp << "}";
    return stream;
}

struct ImageData720p {
    static constexpr const char* IOX2_TYPE_NAME = "ImageData720p";
    uint8_t flip_mode; // 0=none, 1=horizontal, 2=vertical, 3=both
    std::array<uint8_t, 1280 * 720 * 4> image_data;
};

inline std::ostream& operator<<(std::ostream& os, const ImageData720p& v) {
    os << "ImageData720p { image_data_size: " << v.image_data.size() << " bytes"
       << ", image_data[0]: " << static_cast<int>(v.image_data[0]) << " }";
    return os;
}


struct GenericData128B {
    static constexpr const char* IOX2_TYPE_NAME = "GenericData128B";
    std::array<std::uint8_t, 128> data;
};

inline std::ostream& operator<<(std::ostream& os, const GenericData128B& v) {
    os << "GenericData128B { data: [";
    const std::size_t n = std::min<std::size_t>(3, v.data.size());
    for (std::size_t i = 0; i < n; ++i) {
        os << static_cast<int>(v.data[i]) << (i + 1 < n ? ", " : "");
    }
    os << "...] }";
    return os;
}

struct GenericData1KB {
    static constexpr const char* IOX2_TYPE_NAME = "GenericData1KB";
    std::array<std::uint8_t, 1024> data{};
};

inline std::ostream& operator<<(std::ostream& os, const GenericData1KB& v) {
    os << "GenericData1KB { data: [";
    const std::size_t n = std::min<std::size_t>(3, v.data.size());
    for (std::size_t i = 0; i < n; ++i) {
        os << static_cast<int>(v.data[i]) << (i + 1 < n ? ", " : "");
    }
    os << "...] }";
    return os;
}

#endif
