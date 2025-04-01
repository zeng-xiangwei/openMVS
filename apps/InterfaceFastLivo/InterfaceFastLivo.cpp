// #include <OpenMVS/MVS.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"

namespace {
// 读取图片及其位姿数据
namespace MVS_IO {
typedef REAL RealT;
typedef Eigen::Matrix<RealT, 3, 3, Eigen::RowMajor> Mat33;
typedef Eigen::Matrix<RealT, 3, 1> Vec3;

// Structure to model the pinhole camera projection model
struct Camera {
    Mat33 K;  // camera's normalized intrinsics matrix
    uint32_t width;
    uint32_t height;
};
typedef std::vector<Camera> vec_Camera;

// structure describing a pose along the trajectory of a platform
struct Pose {
    Mat33 R;  // pose's rotation matrix
    Vec3 C;   // pose's translation vector
};
typedef std::vector<Pose> vec_Pose;

// structure describing an image
struct Image {
    uint32_t id_camera;  // ID of the associated camera on the associated platform
    uint32_t id_pose;    // ID of the pose of the associated platform
    std::string name;    // image file name
};
typedef std::vector<Image> vec_Image;

struct SfM_Scene {
    vec_Pose poses;      // array of poses
    vec_Camera cameras;  // array of cameras
    vec_Image images;    // array of images
};

/**
 * @brief
 *
 * @param src_dir 图片以及位姿存放的文件夹路径
 * @param pose_file_name 位姿数据文件名，该文件内包括了所有图片的位姿以及对应的图片名
 * @param intrinsic_file_name 相机的内参文件名
 * @return true
 * @return false
 */
bool importScene(const std::string& src_dir, const std::string& pose_file_name,
                 const std::string& intrinsic_file_name, SfM_Scene& result_scene) {
    // 读取相机外参
    std::unordered_map<uint32_t, Camera> cameras_intrinsic;
    std::string intrinsic_file =
        src_dir.back() == '/' ? src_dir + intrinsic_file_name : src_dir + "/" + intrinsic_file_name;
    std::ifstream read_intrinsic(intrinsic_file);
    std::string read_intrinsic_str_line;
    while (std::getline(read_intrinsic, read_intrinsic_str_line)) {
        std::stringstream ss(read_intrinsic_str_line);
        std::cout << "intrinsic: " << ss.str() << std::endl;
        RealT fx, fy, cx, cy;
        uint32_t width, height, camera_id;
        ss >> width >> height >> fx >> fy >> cx >> cy >> camera_id;
        Mat33 intri;
        intri << fx, 0., cx, 0., fy, cy, 0., 0., 1;
        cameras_intrinsic[camera_id] = Camera{intri, width, height};
    }

    // 读取位姿以及图片名
    std::unordered_map<int, int> cameras_id_to_index;
    std::string pose_file =
        src_dir.back() == '/' ? src_dir + pose_file_name : src_dir + "/" + pose_file_name;
    std::ifstream read_pose(pose_file);
    std::string read_pose_str_line;
    while (std::getline(read_pose, read_pose_str_line)) {
        std::stringstream ss(read_pose_str_line);
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        int camera_id;
        std::string pic_name;
        std::cout << "read pose: " << ss.str() << std::endl;
        ss >> q.x() >> q.y() >> q.z() >> q.w() >> t.x() >> t.y() >> t.z() >> camera_id >> pic_name;
        if (cameras_id_to_index.count(camera_id) == 0) {
            cameras_id_to_index[camera_id] = result_scene.cameras.size();
            result_scene.cameras.push_back(Camera{cameras_intrinsic[camera_id]});
        }

        result_scene.poses.push_back(Pose{q.toRotationMatrix(), t});
        std::string pic_full_path =
            src_dir.back() == '/' ? src_dir + pic_name : src_dir + "/" + pic_name;
        std::cout << "pic path: " << pic_full_path << std::endl;
        result_scene.images.push_back(Image{static_cast<uint32_t>(cameras_id_to_index[camera_id]),
                                            static_cast<uint32_t>(result_scene.poses.size() - 1),
                                            pic_full_path});
    }

    return true;
}

bool importPointCloud(const std::string& pcd_file_path, MVS::PointCloud dst_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB> src_cloud;
    pcl::io::loadPCDFile(pcd_file_path, src_cloud);
    if (src_cloud.size() == 0) {
        std::cout << "error: can not read pcd file" << std::endl;
        return false;
    }

    std::cout << "read pcd file success, points size: " << src_cloud.size() << std::endl;

    dst_cloud.points.Reserve(src_cloud.points.size());
    for (const auto& src_point : src_cloud.points) {
        auto& dst_point = dst_cloud.points.AddEmpty();
        dst_point.x = src_point.x;
        dst_point.y = src_point.y;
        dst_point.z = src_point.z;
    }

    return true;
}

bool customSceneToMvsScene(const SfM_Scene& src_scene, MVS::Scene& dst_scene) {
    // convert data from OpenMVG to OpenMVS
    size_t n_cameras = src_scene.cameras.size();
    dst_scene.platforms.Reserve((uint32_t)n_cameras);
    for (const auto& src_camera : src_scene.cameras) {
        MVS::Platform& platform = dst_scene.platforms.AddEmpty();
        platform.name = "platform";
        MVS::Platform::Camera& camera = platform.cameras.AddEmpty();
        camera.K = src_camera.K;
        camera.R = Mat33::Identity();
        camera.C = Vec3::Zero();

        const REAL fScale(REAL(1)/MVS::Camera::GetNormalizationScale(src_camera.width, src_camera.height));
        std::cout << "fScale: " << fScale << std::endl;
        camera.K(0,0) *= fScale;
        camera.K(1,1) *= fScale;
        camera.K(0,2) *= fScale;
        camera.K(1,2) *= fScale;
    }
    size_t n_poses = src_scene.images.size();
    dst_scene.images.Reserve((uint32_t)n_poses);
    int image_id = 0;
    for (const auto& src_image : src_scene.images) {
        const Pose& src_pose = src_scene.poses[src_image.id_pose];
        MVS::Image& image = dst_scene.images.AddEmpty();
        image.name = src_image.name;
        image.ID = image_id++;
        image.platformID = src_image.id_camera;
        MVS::Platform& platform = dst_scene.platforms[image.platformID];
        image.cameraID = src_image.id_camera;
        image.poseID = platform.poses.GetSize();
        image.LoadImage(image.name);
        MVS::Platform::Pose& pose = platform.poses.AddEmpty();
        pose.R = src_pose.R;
        pose.C = src_pose.C;
        std::cout << "image id: " << image.ID << " pose id: " << image.poseID << std::endl;
        std::cout << "pose.R: " << pose.R << std::endl;
        std::cout << "pose.C: " << pose.C << std::endl;
    }

    return true;
}

}  // namespace MVS_IO
}  // namespace

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cout << "Usage: ./InterfaceFastLivo src_dir pose_file_name intrinsic_file_name "
                     "pcd_file_path mvs_file_path\n";
    }
    std::string src_dir = argv[1];
    std::string pose_file_name = argv[2];
    std::string intrinsic_file_name = argv[3];
    std::string pcd_file_path = argv[4];
    std::string mvs_file_path = argv[5];
    MVS::Scene scene;
    MVS_IO::SfM_Scene sfm_scene;

    MVS_IO::importScene(src_dir, pose_file_name, intrinsic_file_name, sfm_scene);
    MVS_IO::customSceneToMvsScene(sfm_scene, scene);

    scene.Save(mvs_file_path);
    return 0;
}
