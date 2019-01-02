#include <graphene/pcl_cloud.hpp>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <charconv>
namespace fs = std::filesystem;

#include <ctre.hpp>

namespace {
    enum class file_version : int {
        v6,
        v7
    };

    enum class file_type : int {
        ascii,
        binary,
        binary_compressed
    };
};

namespace graphene::pcl {

cloud::cloud(const std::filesystem::path& file_path, std::optional<vec4f_t> point_color)
{
    terminate_unless(fs::exists(file_path), "Input PCD file \"{}\" does not exist", file_path.string());

    if (std::ifstream in(file_path.string(), std::ios::in | std::ios::binary); in.good()) {
        file_type type = file_type::ascii;
        file_version version = file_version::v6;
        vec3f_t origin = vec3f_t::Zero();
        Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
        uint32_t data_idx = 0u;
        int width = 0;
        int height = 0;
        int point_step = 0;
        int row_step = 0;

        uint32_t point_count = 0;


        std::vector<int> field_sizes, field_counts;
        std::vector<std::string> field_names;
        std::vector<char> field_types;

        try {
            std::string line;
            while (!in.eof()) {
                std::getline(in, line);
                if (line == "") {
                    continue;
                }

                using namespace ctre::literals;

                if (auto [whole, major, minor] = "VERSION (\\d+)((?:\\.\\d+)*)"_ctre.match(line); whole) {
                    continue;
                }

                if (auto [whole, fields] = "FIELDS (.*)"_ctre.match(line); whole) {
                    for (auto m : ctre::range(fields, "\\w++"_ctre)) {
                        field_names.push_back(std::string(std::string_view{m}));
                    }
                    continue;
                }

                if (auto [whole, fields] = "SIZE (.*)"_ctre.match(line); whole) {
                    for (auto m : ctre::range(fields, "\\w++"_ctre)) {
                        auto v = std::string_view{m};
                        int size;
                        std::from_chars(&v[0], &v[0] + v.size(), size);
                        field_sizes.push_back(size);
                    }
                    continue;
                }

                if (auto [whole, fields] = "TYPE (.*)"_ctre.match(line); whole) {
                    for (auto m : ctre::range(fields, "\\w++"_ctre)) {
                        field_types.push_back(std::string_view{m}[0]);
                    }
                    continue;
                }

                if (auto [whole, fields] = "COUNT (.*)"_ctre.match(line); whole) {
                    for (auto m : ctre::range(fields, "\\w++"_ctre)) {
                        auto v = std::string_view{m};
                        int count;
                        std::from_chars(&v[0], &v[0] + v.size(), count);
                        field_counts.push_back(count);
                    }
                    continue;
                }

                if (auto [whole, w] = "WIDTH (\\d+)"_ctre.match(line); whole) {
                    auto v = w.to_view();
                    std::from_chars(&v[0], &v[0] + v.size(), width);
                    continue;
                }

                if (auto [whole, h] = "HEIGHT (\\d+)"_ctre.match(line); whole) {
                    auto v = h.to_view();
                    std::from_chars(&v[0], &v[0] + v.size(), height);
                    continue;
                }

                if (auto [whole, values] = "VIEWPOINT (.*)"_ctre.match(line); whole) {
                    std::string s(values.to_view());
                    float x, y, z, qw, qx, qy, qz;
                    sscanf(s.c_str(), "%f %f %f %f %f %f %f", &x, &y, &z, &qw, &qx, &qy, &qz);
                    origin = vec3f_t(x, y, z);
                    orientation = Eigen::Quaternionf(qw, qx, qy, qz);
                    transform_.topLeftCorner<3,3>() = orientation.toRotationMatrix();
                    transform_.topRightCorner<3,1>() = origin;
                    continue;
                }

                if (auto [whole, c] = "POINTS (\\d+)"_ctre.match(line); whole) {
                    auto v = c.to_view();
                    std::from_chars(&v[0], &v[0] + v.size(), point_count);
                    continue;
                }

                if (auto [whole, m] = "DATA (\\w+)"_ctre.match(line); whole) {
                    std::string smode(m.to_view());
                    if (smode == "binary") {
                        type = file_type::binary;
                    } else if (smode == "ascii") {
                        type = file_type::ascii;
                    } else if (smode == "binary_compressed") {
                        type = file_type::binary_compressed;
                    } else {
                        fail("Invalid PCD file mode \"{}\"", smode);
                    }
                    break;
                }
            }


            mat_ = data_matrix_t(point_count, 10);
            indices_.resize(point_count);

            typedef union
            {
                union
                {
                    struct
                    {
                        uint8_t b;
                        uint8_t g;
                        uint8_t r;
                        uint8_t a;
                    };
                    float rgb;
                };
                uint32_t rgba;
            } pcl_rgba_t;
            if (type == file_type::binary) {
                int point_size = 0;
                int off_x = -1, off_y = -1, off_z = -1;
                int off_nx = -1, off_ny = -1, off_nz = -1;
                int off_rgba = -1, off_curv = -1;
                for (uint32_t i = 0; i < field_names.size(); ++i) {
                    if (field_names[i] == "x") {
                        off_x = point_size;
                    } else if (field_names[i] == "y") {
                        off_y = point_size;
                    } else if (field_names[i] == "z") {
                        off_z = point_size;
                    } else if (field_names[i] == "normal_x") {
                        off_nx = point_size;
                    } else if (field_names[i] == "normal_y") {
                        off_ny = point_size;
                    } else if (field_names[i] == "normal_z") {
                        off_nz = point_size;
                    } else if (field_names[i] == "rgba") {
                        off_rgba = point_size;
                    } else if (field_names[i] == "curvature") {
                        off_curv = point_size;
                    }
                    point_size += field_counts[i] * field_sizes[i];
                }

                bool has_pos = off_x >= 0 && off_y >= 0 && off_z >= 0;
                bool has_nrm = off_nx >= 0 && off_ny >= 0 && off_nz >= 0;
                bool has_rgba = off_rgba >= 0;
                bool has_curv = off_curv >= 0;
                int64_t cur = in.tellg();
                in.seekg(0, in.end);
                int64_t data_size = in.tellg() - cur;
                in.seekg(cur, in.beg);

                char* buffer = new char[point_size];
                pcl_rgba_t prgba;
                packed_color_t grgba;
                for (uint32_t i = 0; i < point_count; ++i) {
                    in.read(buffer, point_size);
                    float* mat_off = mat_.data() + i*10;
                    if (has_pos) {
                        memcpy((char*) (mat_off + 0), buffer + off_x, sizeof(float));
                        memcpy((char*) (mat_off + 1), buffer + off_y, sizeof(float));
                        memcpy((char*) (mat_off + 2), buffer + off_z, sizeof(float));
                    }
                    if (has_nrm) {
                        memcpy((char*) (mat_off + 3), buffer + off_nx, sizeof(float));
                        memcpy((char*) (mat_off + 4), buffer + off_ny, sizeof(float));
                        memcpy((char*) (mat_off + 5), buffer + off_nz, sizeof(float));
                    }
                    if (has_rgba) {
                        memcpy((char*) &prgba.rgba, buffer + off_rgba, sizeof(uint32_t));
                        grgba.r = prgba.r;
                        grgba.g = prgba.g;
                        grgba.b = prgba.b;
                        grgba.a = prgba.a;
                        if (point_color) {
                            vec4f_t pc = *point_color;
                            grgba.r = static_cast<uint8_t>(pc[0] * 255.f);
                            grgba.g = static_cast<uint8_t>(pc[1] * 255.f);
                            grgba.b = static_cast<uint8_t>(pc[2] * 255.f);
                            grgba.a = static_cast<uint8_t>(pc[3] * 255.f);
                        }
                        mat_(i, 6) = grgba.packed;
                    }
                    if (has_curv) {
                        memcpy((char*) (mat_off + 9), buffer + off_curv, sizeof(float));
                    }
                    mat_(i, 7) = mat_(i, 8) = 0.f; // uv
                    indices_[i] = i;
                }
            } else if (type == file_type::ascii) {
                int off_x = -1, off_y = -1, off_z = -1;
                int off_nx = -1, off_ny = -1, off_nz = -1;
                int off_rgba = -1, off_curv = -1;
                for (uint32_t i = 0; i < field_names.size(); ++i) {
                    if (field_names[i] == "x") {
                        off_x = i;
                    } else if (field_names[i] == "y") {
                        off_y = i;
                    } else if (field_names[i] == "z") {
                        off_z = i;
                    } else if (field_names[i] == "normal_x") {
                        off_nx = i;
                    } else if (field_names[i] == "normal_y") {
                        off_ny = i;
                    } else if (field_names[i] == "normal_z") {
                        off_nz = i;
                    } else if (field_names[i] == "rgba") {
                        off_rgba = i;
                    } else if (field_names[i] == "curvature") {
                        off_curv = i;
                    }
                }
                pcl_rgba_t prgba;
                packed_color_t grgba;
                for (uint32_t i = 0; i < point_count; ++i) {
                    std::string line;
                    std::getline(in, line);
                    std::istringstream buffer(line);
                    std::vector<std::string> tokens{std::istream_iterator<std::string>(buffer),
                                                    std::istream_iterator<std::string>()};
                    if (off_x >= 0 && off_y <= 0 && off_z >= 0) {
                        mat_(i, 0) = std::stof(tokens[off_x]);
                        mat_(i, 1) = std::stof(tokens[off_y]);
                        mat_(i, 2) = std::stof(tokens[off_z]);
                    }
                    if (off_nx >= 0) {
                        mat_(i, 3) = std::stof(tokens[off_nx]);
                    }
                    if (off_ny >= 0) {
                        mat_(i, 4) = std::stof(tokens[off_ny]);
                    }
                    if (off_nz >= 0) {
                        mat_(i, 5) = std::stof(tokens[off_nz]);
                    }
                    if (off_rgba >= 0) {
                        prgba.rgba = static_cast<uint32_t>(std::stoul(tokens[off_rgba]));
                        grgba.r = prgba.r;
                        grgba.g = prgba.g;
                        grgba.b = prgba.b;
                        grgba.a = prgba.a;
                        if (point_color) {
                            vec4f_t pc = *point_color;
                            grgba.r = static_cast<uint8_t>(pc[0] * 255.f);
                            grgba.g = static_cast<uint8_t>(pc[1] * 255.f);
                            grgba.b = static_cast<uint8_t>(pc[2] * 255.f);
                            grgba.a = static_cast<uint8_t>(pc[3] * 255.f);
                        }
                        mat_(i, 6) = grgba.packed;
                    }
                    mat_(i, 7) = mat_(i, 8) = 0.f; // uv
                    if (off_curv >= 0) {
                        mat_(i, 9) = std::stof(tokens[off_curv]);
                    }
                    indices_[i] = i;
                }
            } else {
                fail("Only PCD file modes binary and ascii are supported");
            }
        } catch (std::exception& ex) {
            fail(ex.what());
        }

        terminate_unless(point_count, "PCD File contains no points");

        if (!height) {
            height = 1;
            if (!width) {
                width = point_count;
            }
        } else {
            terminate_unless(width, "Pointcloud has height but no width");
        }

        terminate_unless(width * height == point_count, "height*width ({}x{}) != point count ({})", width, height, point_count);
    } else {
        fail("Unable to open file {} for reading", file_path.string());
    }
}

std::optional<std::vector<uint8_t>>
cloud::texture() const
{
    return std::nullopt;
}

vec2i_t
cloud::texture_size() const
{
    return vec2i_t::Zero();
}

render_mode_t
cloud::render_mode() const
{
    return render_mode_t::splats;
}

bool
cloud::shaded() const
{
    return true;
}

renderable::data_matrix_t
cloud::data_matrix() const
{
    return mat_;
}

std::vector<uint32_t>
cloud::vertex_indices() const
{
    return indices_;
}

} // graphene::pcl
