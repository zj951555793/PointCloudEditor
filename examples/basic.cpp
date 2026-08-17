#include <JMEngine/JMEngine.h>
#include <JMEngine/PointCloudIO.h>

#include <iostream>
#include <memory>
#include <string>

int main(int argc, char* argv[]) {
    using namespace JMEngine;

    std::shared_ptr<PointCloud> cloud;

    // 可以直接传入 OBJ/PLY：
    //   JMEngine_example model.ply
    //   JMEngine_example model.obj
    if (argc > 1) {
        std::string error;
        cloud = PointCloudIO::load(argv[1], &error);
        if (!cloud) {
            std::cerr << "加载失败: " << error << '\n';
            return 2;
        }
    } else {
        // 未指定文件时，构造 3 个点做最小功能验证。
        PointCloud::Container pts;
        pts.push_back({{-0.5f, 0.0f, 0.0f}, 0xff0000ffu, PointValid});
        pts.push_back({{0.0f, 0.0f, 0.0f}, 0x00ff00ffu, PointValid});
        pts.push_back({{0.5f, 0.0f, 0.0f}, 0x0000ffffu, PointValid});
        cloud = std::make_shared<PointCloud>(std::move(pts));
    }

    Engine editor(cloud);
    std::cout << "初始点数 = " << cloud->size() << '\n';

    // 至少有 2 个点时，演示删除和 Undo。
    if (cloud->size() >= 2) {
        editor.select({1});
        editor.deleteSelection();
        std::cout << "删除后有效点数 = " << cloud->activeCount() << '\n';

        editor.undo();
        std::cout << "Undo 后有效点数 = " << cloud->activeCount() << '\n';
    }

    // 演示全点云 X 方向平移 1.0。
    Mat4f transform = Mat4f::identity();
    transform.m[12] = 1.0f;
    editor.clearSelection();
    editor.transform(transform);

    if (!cloud->empty()) {
        std::cout << "变换后 point0.x = " << cloud->at(0).position.x << '\n';
    }

    // compact 会物理删除软删除点并返回 PointId 映射。
    const auto mapping = editor.compact();
    std::cout << "compact 后 size = " << cloud->size() << '\n';
    std::cout << "old->new 映射数量 = " << mapping.size() << '\n';

    return 0;
}
