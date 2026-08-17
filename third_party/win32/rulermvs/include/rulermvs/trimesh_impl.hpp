#ifndef _RULERMVS_CORE_TRIMESH_IMPL_HPP_
#define _RULERMVS_CORE_TRIMESH_IMPL_HPP_
namespace rulermvs
{

template <typename Float, typename Index>
std::vector<Index> SimpleTriMesh_<Float, Index>::checkUnusedPoints() const
{
    const auto        pt_num = static_cast<Index>(points.size());
    std::vector<bool> unused_flags(points.size(), true);
    for (size_t i = 0; i < tript_inds.size(); ++i) {
        auto& tri_ind = tript_inds[i];
        if (tri_ind.s[0] >= 0 && tri_ind.s[0] < pt_num)
            unused_flags[tri_ind.s[0]] = false;
        if (tri_ind.s[1] >= 0 && tri_ind.s[1] < pt_num)
            unused_flags[tri_ind.s[1]] = false;
        if (tri_ind.s[2] >= 0 && tri_ind.s[2] < pt_num)
            unused_flags[tri_ind.s[2]] = false;
    }
    std::vector<Index> unused_ptinds;
    for (Index i = 0; i < (Index)unused_flags.size(); ++i)
        if (unused_flags[i]) unused_ptinds.emplace_back(i);
    return unused_ptinds;
}
template <typename Float, typename Index>
std::vector<Index> SimpleTriMesh_<Float, Index>::checkUnvalidPoints() const
{
    std::vector<Index> unvalid_ptinds;
    for (Index i = 0; i < (Index)points.size(); ++i) {
        const auto& pt = points[i];
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) ||
            std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z))
            unvalid_ptinds.emplace_back(i);
    }
    return unvalid_ptinds;
}
template <typename Float, typename Index>
std::vector<Index> SimpleTriMesh_<Float, Index>::checkUnvalidTripts() const
{
    std::vector<Index> unvalid_triinds;
    // 将越界三角面索引排除
    Index pt_num = static_cast<Index>(points.size());
    for (Index i = 0; i < (Index)tript_inds.size(); ++i) {
        const auto& tri_ind = tript_inds[i];
        if (tri_ind.s[0] < 0 || tri_ind.s[0] >= pt_num || tri_ind.s[1] < 0 ||
            tri_ind.s[1] >= pt_num || tri_ind.s[2] < 0 ||
            tri_ind.s[2] >= pt_num) {
            unvalid_triinds.emplace_back(i);
        }
    }
    return unvalid_triinds;
}
template <typename Float, typename Index>
SimpleTriMesh_<Float, Index>& SimpleTriMesh_<Float, Index>::removePoints(
    const std::vector<bool>& inliers)
{
    assert(inliers.size() == getPointNum());
    std::vector<Index> pt_inds(inliers.size(), (Index)-1);
    if (hasNormal()) {
        Index num = 0;
        for (size_t i = 0; i < inliers.size(); ++i) {
            if (!inliers[i]) {
                pt_inds[i]     = num;
                points[num]    = points[i];
                normals[num++] = normals[i];
            }
        }
        points.resize(num);
        normals.resize(num);
    } else {
        Index num = 0;
        for (size_t i = 0; i < inliers.size(); ++i) {
            if (!inliers[i]) {
                pt_inds[i]    = num;
                points[num++] = points[i];
            }
        }
        points.resize(num);
    }
    // 更新面索引
    bool update_tript = false;
    for (size_t i = 0; i < tript_inds.size(); ++i) {
        auto& tri_ind = tript_inds[i];
        tri_ind[0]    = pt_inds[tri_ind[0]];
        tri_ind[1]    = pt_inds[tri_ind[1]];
        tri_ind[2]    = pt_inds[tri_ind[2]];
        if (!update_tript &&
            (tri_ind[0] == -1 || tri_ind[1] == -1 || tri_ind[2] == -1))
            update_tript = true;
    }
    if (update_tript) removeUnvalidTripts();
    return *this;
}

template <typename Float, typename Index>
SimpleTriMesh_<Float, Index>& SimpleTriMesh_<Float, Index>::removePoints(
    const std::vector<Index>& pt_inds)
{
    const size_t      pt_num = getPointNum();
    std::vector<bool> inliers(pt_num, false);
    for (size_t i = 0; i < pt_inds.size(); ++i) {
        auto& ptind = pt_inds[i];
        if (ptind >= 0 && ptind < pt_num) inliers[ptind] = true;
    }
    return removePoints(inliers);
}

template <typename Float, typename Index>
SimpleTriMesh_<Float, Index>& SimpleTriMesh_<Float, Index>::removeTripts(
    const std::vector<Index>& tri_inds)
{
    if (!tri_inds.empty()) {
        const size_t      tri_num = getTriFaceNum();
        std::vector<bool> inliers(tri_num, true);
        for (size_t i = 0; i < tri_inds.size(); ++i) {
            auto& ind = tri_inds[i];
            if (ind >= 0 && ind < tri_num) inliers[ind] = false;
        }
        Index num = 0;
        for (size_t i = 0; i < tri_num; ++i)
            if (inliers[i]) tript_inds[num++] = tript_inds[i];
        tript_inds.resize(num);
    }
    return *this;
}

template <typename Float, typename Index> SimpleTriMesh_<Float, Index>&
SimpleTriMesh_<Float, Index>::removeIsolatedTripts(Index min_group)
{
    Index              group_num = 0;
    std::vector<Index> group_inds(points.size(), 0);
    std::vector<Index> tmp_inds;
    tmp_inds.reserve(group_inds.size());
    auto vertex_faces = incidentTripts();
    for (size_t i = 0; i < points.size(); ++i) {
        if (!group_inds[i]) group_inds[i] = ++group_num;
        auto& tri_inds = vertex_faces[i];
        tmp_inds.insert(tmp_inds.end(), tri_inds.begin(), tri_inds.end());
        while (!tmp_inds.empty()) {
            auto tri_ind = tript_inds[tmp_inds.back()];
            tmp_inds.pop_back();
            if (!group_inds[tri_ind.s[0]]) {
                group_inds[tri_ind.s[0]] = group_num;
                auto& tmp_tri_inds       = vertex_faces[tri_ind.s[0]];
                tmp_inds.insert(
                    tmp_inds.end(), tmp_tri_inds.begin(), tmp_tri_inds.end());
            }
            if (!group_inds[tri_ind.s[1]]) {
                group_inds[tri_ind.s[1]] = group_num;
                auto& tmp_tri_inds       = vertex_faces[tri_ind.s[1]];
                tmp_inds.insert(
                    tmp_inds.end(), tmp_tri_inds.begin(), tmp_tri_inds.end());
            }
            if (!group_inds[tri_ind.s[2]]) {
                group_inds[tri_ind.s[2]] = group_num;
                auto& tmp_tri_inds       = vertex_faces[tri_ind.s[2]];
                tmp_inds.insert(
                    tmp_inds.end(), tmp_tri_inds.begin(), tmp_tri_inds.end());
            }
        }
    }
    std::vector<Index> group_counts(group_num + 1, 0);
    for (size_t i = 0; i < group_inds.size(); ++i)
        group_counts[group_inds[i]]++;

    // Index min_group = min_group;
    for (size_t i = 1; i < group_counts.size(); ++i)
        group_counts[i] = group_counts[i] > min_group;
    group_counts[0] = 0;
    for (size_t i = 0; i < group_inds.size(); ++i)
        group_inds[i] = group_counts[group_inds[i]];

    std::vector<Index> unvalid_tripts;
    for (size_t i = 0; i < group_inds.size(); ++i) {
        if (!group_inds[i])
            unvalid_tripts.insert(unvalid_tripts.end(), vertex_faces[i].begin(),
                vertex_faces[i].end());
    }
    std::sort(unvalid_tripts.begin(), unvalid_tripts.end());
    unvalid_tripts.erase(unique(unvalid_tripts.begin(), unvalid_tripts.end()),
        unvalid_tripts.end());
    removeTripts(unvalid_tripts);
    return *this;
}

/// @param min_point_faces point连接的面片数量最小阈值
template <typename Float, typename Index> SimpleTriMesh_<Float, Index>&
SimpleTriMesh_<Float, Index>::erodeEdgeTripts(Index mesh_erode_times, Index min_point_faces)
{
	std::vector<int> point_faces(points.size(), 0);
    std::vector<std::vector<int>> point_faces_num(points.size(), std::vector<int>(6, -1));

    for (int i = 0; i < tript_inds.size(); i++)
    {
        auto point_nums = tript_inds[i];
        for (int j = 0; j < 3; j++)
        {
            auto point_num = point_nums[j];
            auto& count = point_faces[point_num];
            point_faces_num[point_num][count++] = i;
        }
    }

    std::vector<size_t> erode_points(points.size(), 0), erode_faces(tript_inds.size(), 0);
    std::vector<int> point_faces2, point_mask(points.size(), 1), tript_mask(tript_inds.size(), 1);
    int point_count = 0, tript_count = 0;
    point_faces2.assign(point_faces.begin(), point_faces.end());
    for (int i = 0; i < mesh_erode_times; i++)
    {
        for (int j = 0; j < point_mask.size(); j++) 
        {
            if (point_mask[j] && point_faces[j] <= min_point_faces)
            {
                erode_points[point_count++] = j;
                point_mask[j] = 0;  //多次腐蚀时起作用
                for (int k = 0; k < 6; k++)
                {
                    auto tript_num = point_faces_num[j][k];
                    if (tript_num < 0 || !tript_mask[tript_num])
                        continue;
                    erode_faces[tript_count++] = tript_num;
                    tript_mask[tript_num]      = 0;
                    auto point_nums = tript_inds[tript_num];
                    for (int m = 0; m < 3; m++)
                    {
                        point_faces2[point_nums[m]]--;
                    }
                }
            }
        }
        point_faces.assign(point_faces2.begin(), point_faces2.end());
    }
    erode_points.resize(point_count), erode_faces.resize(tript_count);
    removePoints(erode_points);
    return *this;
}

template <typename Float, typename Index>
std::vector<std::vector<Index>> SimpleTriMesh_<Float, Index>::incidentTripts()
    const
{
    std::vector<std::vector<Index>> vertex_faces(points.size());
    for (Index i = 0; i < (Index)tript_inds.size(); ++i) {
        const auto& tri_ind = tript_inds[i];
        vertex_faces[tri_ind.s[0]].emplace_back(i);
        vertex_faces[tri_ind.s[1]].emplace_back(i);
        vertex_faces[tri_ind.s[2]].emplace_back(i);
    }
    return vertex_faces;
}

template <typename Float, typename Index>
std::vector<std::vector<Index>> SimpleTriMesh_<Float, Index>::adjacencyTripts()
    const
{
    std::vector<std::vector<Index>> vertex_faces = incidentTripts();
    std::vector<std::vector<Index>> adjacency_faces;
    for (size_t i = 0; i < tript_inds.size(); ++i) {
        const auto& tri_ind  = tript_inds[i];
        auto&       adj_inds = adjacency_faces[i];
        const auto& faces0   = vertex_faces[tri_ind[0]];
        const auto& faces1   = vertex_faces[tri_ind[1]];
        const auto& faces2   = vertex_faces[tri_ind[2]];
        adj_inds.assign(faces0.begin(), faces0.end());
        adj_inds.insert(adj_inds.end(), faces1.begin(), faces1.end());
        adj_inds.insert(adj_inds.end(), faces2.begin(), faces2.end());
        std::sort(adj_inds.begin(), adj_inds.end());
        adj_inds.erase(
            unique(adj_inds.begin(), adj_inds.end()), adj_inds.end());
    }
    return adjacency_faces;
}

template <typename Float, typename Index>
std::vector<Index> SimpleTriMesh_<Float, Index>::groupComponents(
    std::vector<Index>& group_inds) const
{
    Index group_num = 0;
    group_inds.resize(points.size(), 0);
    std::vector<Index> tmp_inds;
    tmp_inds.reserve(group_inds.size());
    auto vertex_faces = incidentTripts();
    for (size_t i = 0; i < points.size(); ++i) {
        if (!group_inds[i]) group_inds[i] = ++group_num;
        auto& tri_inds = vertex_faces[i];
        tmp_inds.insert(tmp_inds.end(), tri_inds.begin(), tri_inds.end());
        while (!tmp_inds.empty()) {
            auto tri_ind = tript_inds[tmp_inds.back()];
            tmp_inds.pop_back();
            if (!group_inds[tri_ind[0]]) {
                group_inds[tri_ind[0]] = group_num;
                auto& tmp_tri_inds     = vertex_faces[tri_ind[0]];
                tmp_inds.insert(
                    tmp_inds.end(), tmp_tri_inds.begin(), tmp_tri_inds.end());
            }
            if (!group_inds[tri_ind[1]]) {
                group_inds[tri_ind[1]] = group_num;
                auto& tmp_tri_inds     = vertex_faces[tri_ind[1]];
                tmp_inds.insert(
                    tmp_inds.end(), tmp_tri_inds.begin(), tmp_tri_inds.end());
            }
            if (!group_inds[tri_ind[2]]) {
                group_inds[tri_ind[2]] = group_num;
                auto& tmp_tri_inds     = vertex_faces[tri_ind[2]];
                tmp_inds.insert(
                    tmp_inds.end(), tmp_tri_inds.begin(), tmp_tri_inds.end());
            }
        }
    }
    std::vector<Index> group_counts(group_num + 1, 0);
    for (size_t i = 0; i < group_inds.size(); ++i)
        group_counts[group_inds[i]]++;
    return group_counts;
}
}  // namespace rulermvs
#endif  // _RULERMVS_CORE_TRIMESH_IMPL_HPP_