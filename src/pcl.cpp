#include <graphene/pcl.hpp>

namespace graphene::pcl {

std::shared_ptr<cloud>
load_cloud(const std::filesystem::path& file_path, std::optional<vec4f_t> point_color)
{
    return std::make_shared<cloud>(file_path, point_color);
}

}  // namespace graphene::pcl
