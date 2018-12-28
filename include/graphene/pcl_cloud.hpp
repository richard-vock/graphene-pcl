#pragma once

#include <graphene/renderable.hpp>

namespace graphene::pcl {

class cloud : public renderable
{
public:
    cloud(const std::filesystem::path& file_path, std::optional<vec4f_t> point_color = std::nullopt);

    virtual ~cloud() = default;

    virtual std::optional<std::vector<uint8_t>>
    texture() const;

    virtual vec2i_t
    texture_size() const;

    virtual render_mode_t
    render_mode() const;

    virtual bool
    shaded() const;

    virtual data_matrix_t
    data_matrix() const;

    virtual std::vector<uint32_t>
    vertex_indices() const;

protected:
    data_matrix_t mat_;
    std::vector<uint32_t> indices_;
};

} // graphene::pcl
