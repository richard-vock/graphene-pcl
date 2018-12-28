#pragma once

#include "pcl_cloud.hpp"

namespace graphene::pcl {

std::shared_ptr<cloud>
load_cloud(const std::filesystem::path& file_path, std::optional<vec4f_t> point_color = std::nullopt);

} // graphene::pcl
